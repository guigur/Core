# PEBB driver bench

Build and flash the `PEBB` branch with the `USB` PlatformIO environment. Open
the USB serial console at 115200 baud, press Enter, and type `help`.

The bench replaces the former LED-blink `src/main.cpp`. Commands use zero-based
IDs: AFEs 0–9 follow `pebb_afe_id`, and HB modules 0–2 follow `pebb_hb_id`.

| Command | What it checks or changes |
| --- | --- |
| `scan` | Per-AFE driver readiness, independent TCA9555 and expected MCP4728 address responses, plus a physical TCA9555 scan on both AFE I²C branches |
| `dacscan` | Separate MCP4728 probes at `0x60`–`0x67` on both branches, even if AFE drivers failed initialization |
| `feederdiag` | Read-only probe of feeder EEPROM `0x50` on U805 channel 2, U814 `0x71` across U804, and U837/U838 behind U814 |
| `hbdiag` | Read-only T117B ROM probes at `0x40`–`0x43` and 24C02S `0x50` on U814 channels 2–4, with the selected mux mask and driver readiness |
| `i2crecover` | Pulse I2C1 SCL on PA15/PB7 using Zephyr's STM32 bus recovery, then probe root mux U805 at `0x70` |
| `afe 2 provision` | Detect V2_LOW DAC's existing address through LDAC selection and program its configured address `0x64` only if needed |
| `afe 2 provision 0x60` | Manual override: require the detected V2_LOW address to be `0x60` before programming `0x64` |
| `afe 6 calibrate` | Sweep all eight I2_LOW gains using its self-test DACs and PA1/ADC2; store fit coefficients in RAM |
| `afe 6 calibrate 4` | Calibrate only gain index 4 (nominal x1) |
| `afe 6 calstatus` | Print all valid RAM fits: raw slope, intercept, R² and gain error |
| `mock start 6 sine 200 50 4` | Generate a sine on I2_LOW with ±200 mV SELF+ differential, gain index 4 and 50 ms per point |
| `mock start all triangle 100 100 4` | Generate the same triangle on every READY AFE |
| `mock read` | Print one CSV snapshot of all ten AFE OUT ADC inputs, including absent slots |
| `mock stream 500`, `mock stream 0` | Change the automatic CSV print period or disable printing while the waveform keeps running |
| `mock status`, `mock stop` | Inspect mock state or stop and reset the selected AFEs |
| `test all` | Read-only checks of PEBB controls/fan status, every AFE GPIO/DAC, every HB temperature/alert/module record, and feeder EEPROM/data |
| `test pebb`, `test feeder` | Read-only checks of one subsystem |
| `test afe 0`, `test afe all` | Read one or all AFE GPIO registers, DAC RAM/EEPROM and INT status |
| `test hb 0`, `test hb all` | Read one or all HB temperatures, alerts and EEPROM records |
| `hb alert status` | Show the six-channel 28 °C alarm mask, valid readings and configured sensor mask |
| `exercise afe 0`, `exercise afe all` | Disconnect stages, set gain index 0 and bandwidth index 7, select external inputs, toggle LED, rewrite the current DAC A RAM value and read it back |
| `exercise all` | Run `test all`, toggle/restore CAN/RS485 termination and exercise all AFEs |

`test all` remains a digital/readback check. `afe ID calibrate` measures the
analog path with the onboard DAC stimulus and STM32 ADC; absolute accuracy
still depends on their references. An erased HB EEPROM is shown
as blank; an unexpected record version or I²C failure is reported as an error.

## AFE automatic gain calibration

Run `scan` first and choose an AFE reported `READY`. Disconnect external
high-energy input signals before calibration. The command selects both self
inputs, connects the gain and bandwidth stages, and holds the output-offset
DAC at the Devicetree startup value (3976 mV by default). For each gain it
drives nine differential points with SELF+ around 2500 mV and SELF- fixed at
2500 mV. The STM32 ADC discards its first sample and averages 16 conversions
at each point. The fit is `OUT_mV = intercept_mV + slope * (SELF+ - SELF-)`.

The command rejects a measured OUT over 2200 mV, a slope outside ±30% of
nominal, R² below 0.97 or an intercept outside 100–2100 mV. It resets the AFE
to its safe startup state afterwards. No DAC EEPROM write occurs. Coefficients
are kept per AFE and gain in RAM until reset; use `afe ID calstatus` to inspect
them. Application code can use `afe_get_gain_calibration()` and
`afe_correct_output_mv()` to compensate an ADC reading. Keep the output-offset
DAC at the same startup setting when applying those coefficients.

| AFE ID | OUT net | STM32 pin | ADC channel |
| ---: | --- | --- | --- |
| 0 V_HIGH | `/VHigh_{MCU}` | PC2 | ADC1_IN8 |
| 1 V1_LOW | `/V1Low_{MCU}` | PC0 | ADC1_IN6 |
| 2 V2_LOW | `/V2Low_{MCU}` | PA0 | ADC2_IN1 |
| 3 V3_LOW | `/V3Low_{MCU}` | PA7 | ADC2_IN4 |
| 4 I_HIGH | `/IHigh_{MCU}` | PC3 | ADC1_IN9 |
| 5 I1_LOW | `/I1Low_{MCU}` | PC1 | ADC1_IN7 |
| 6 I2_LOW | `/I2Low_{MCU}` | PA1 | ADC2_IN2 |
| 7 I3_LOW | `/I3Low_{MCU}` | PB11 | ADC1_IN14 |
| 8 ANALOG1 | `/SIN_{MCU}` | PC5 | ADC2_IN11 |
| 9 ANALOG2 | `/COS_{MCU}` | PA4 | ADC2_IN17 |

These connections come from the current PEBB KiCad netlist. Zephyr's STM32
ADC conversion uses the configured 3300 mV reference; verify VDDA if absolute
mV accuracy matters. PA4 is also DAC1_OUT1 on SPIN: do not start DAC1 channel 1
while calibrating ANALOG2.

## AFE mock waveform and ten-channel ADC readout

`mock start ID|all sine|square|triangle AMP_MV STEP_MS [GAIN]` configures the
selected READY AFEs for self-test input, BW7 and the chosen gain. The waveform
changes SELF+ around 2500 mV while SELF- stays at 2500 mV. There are 32
points per cycle; `STEP_MS` is at least 20 ms and the actual cycle also
includes I²C writes and ADC reads. Gain defaults to index 4 (x1). Amplitude
is limited for each gain so the nominal OUT movement stays near 300 mV.

The bench prints `AFE_OUT_MV,uptime_ms,ready_mask,V_HIGH,...,ANALOG2` followed
by CSV values in mV every 500 ms by default. `mock read` prints one snapshot
even with no waveform running. The `ready_mask` marks AFE drivers that are
READY; ADC readings for absent slots may float and are diagnostic only.
`mock stream PERIOD_MS` changes the print interval (`0` disables it).

The dedicated mock worker checks every selected OUT against 2200 mV after each point. An ADC
or AFE write error, or an over-range reading, stops the waveform and resets
all selected AFEs. `mock stop` does the same on request. While mock is running,
the bench rejects other AFE control, provisioning, calibration, exercise and
I²C recovery commands. Use `mock stop` before those operations.

## HB temperature alarm and LED

The bench starts monitoring the HS and LS T117B on each READY HB at boot.
It configures each sensor's volatile hardware ALERT registers for a 28 °C
upper threshold and 27 °C lower threshold, with an active-low ALERT output.
No sensor EEPROM is written. Configuration retries if a READY sensor does not
respond. The bench also compares the six measured temperatures with the same
thresholds every 500 ms; this keeps the LED indication stable even if a
temperature read clears the sensor's ALERT output. One or more channels at or
above 28 °C make the SPIN user LED on PA5 blink once per second. The LED stops
blinking once all alarmed channels fall to 27 °C or below. A failed reading
does not clear an existing alarm.

Use `hb alert status` to see `active_mask` (bit 0/1 = HB0 HS/LS, bit 2/3 =
HB1 HS/LS, bit 4/5 = HB2 HS/LS), `valid_mask` for successful recent readings,
and `configured_HB_mask` for completed hardware ALERT configuration. `test hb
all` continues to show each sensor's measured temperature and raw ALERT pin.
The raw pin can be inactive after a temperature read has acknowledged an
alert; the software alarm mask controls the blinking LED.

## Feeder I²C wiring

The feeder schematic's J101 pin 1 (SDA), pin 2 (SCL), pins 3–4 (DGND),
pin 5 (`+12V_DGND`) and pin 6 (`+5V_DGND`) match PEBB J811 pin-for-pin.
Feeder EEPROM U101 is at `0x50` on **U805 channel 2**, before U804. The
PEBB power blade U809 carries another I²C pair on the PGND side, exposed by
U814 channel 0, but it is not wired to feeder U101 in these schematics.

`feederdiag` reads U805 itself at `0x70`, U839 through channel 1, and the
selected channel mask after probing feeder U101. A mask of `0x04` confirms
that U805 selected channel 2; it does not prove that J811/J101 or the EEPROM
responded. The command also tests the isolated U814 path separately. If
`0x50` responds on U805 channel 2
but the feeder driver is still `NOT READY`, reboot the MCU with the feeder
power already applied: Zephyr device initialization runs at boot. U838 being
`NOT READY` indicates a separate issue on the U804/U814 PGND side; it does
not imply that the feeder EEPROM on J811 is absent.

If I2C1 is stuck, run `i2crecover`, then `feederdiag` and `scan`. The shield
enables `CONFIG_I2C_STM32_BUS_RECOVERY` with open-drain PA15 (SCL) and PB7
(SDA). Recovery only attempts to release the electrical bus; it does not
rerun Zephyr initialization for devices that failed at boot. PA15 must still
be selected by U726. A device holding SDA low continuously, or an unpowered
downstream branch, may require a power cycle after the electrical fault is
removed.

## Diagnosing AFE address conflicts

The updated PEBB schematic places five AFE connectors on each U805 channel.
The connector A0/A1/A2 straps set the TCA9555 address as follows:

| U805 channel | Slot (connector) | TCA9555 | Expected MCP4728 |
| --- | --- | --- | --- |
| 0 | I2_LOW (J823) | `0x20` | `0x60` |
| 0 | V3_LOW (J830) | `0x21` | `0x61` |
| 0 | ANALOG1 (J825, AFE806) | `0x22` | `0x62` |
| 0 | I3_LOW (J824) | `0x23` | `0x63` |
| 0 | V2_LOW (J829) | `0x24` | `0x64` |
| 1 | V1_LOW (J828) | `0x20` | `0x60` |
| 1 | V_HIGH (J827) | `0x21` | `0x61` |
| 1 | I1_LOW (J831) | `0x22` | `0x62` |
| 1 | ANALOG2 (J826, AFE805) | `0x23` | `0x63` |
| 1 | I_HIGH (J822) | `0x24` | `0x64` |

U839 is also on channel 1 at `0x27`. The MCP4728 addresses in the last column
are the **software configuration**, not connector straps: each DAC stores its
address in its own EEPROM and normally starts at `0x60`. Each fitted board
must have its DAC programmed to the expected address before two boards with
the same DAC address share a channel.
ANALOG1/ANALOG2 follow the PEBB PCB silkscreen: AFE806 at (67,49) is
ANALOG1, and AFE805 at (76,72) is ANALOG2. Their analog connectors are J819
and J820, respectively.

`scan` and `dacscan` list **actual address responses on each mux channel**.
The per-AFE `scan` row starts with the numeric ID used by `afe ID ...` and
reports `ACK`, `NO` or `ERROR` separately for the
TCA9555 and the **configured** MCP4728 address; `ACK_BAD` means the DAC
answered but its 24-byte readback is implausible. If a DAC still has its
factory `0x60` address in a slot expecting another address, the row shows
`NO` for that DAC and `dacscan` shows the actual `0x60` response.
The USB terminal uses ANSI colors in `scan`: white text, green for ready/ACK
states, and red for not-ready/no-response/error states.
The driver `READY` column shows which configured Zephyr device initialized; it
does not prove which physical slot responded. If only `0x20` and `0x60`
respond while a board is fitted in I2_LOW, that is the expected TCA9555 address
for this slot; V2_LOW should respond at `0x24` on the same channel. If two boards on one
channel still have DAC `0x60`, their readback can be corrupted or appear as
one responder. Probe with one AFE fitted at a time, and save both channel
scans for each placement. A responding address is labelled with the slot that
would be expected from the schematic, but that label is not a physical
identification. Do not infer uniqueness from a single ACK. The bench uses
immediate Zephyr logging so scan lines are not dropped by the default 1 KiB
deferred log buffer.

AFE initialization programs and verifies the TCA9555 first, with gain and
bandwidth paths disconnected. It then checks and initializes the MCP4728.
An AFE whose expander responds but whose DAC is absent at the configured
address remains `NOT READY`, although its GPIO configuration register should
read `90 0c` rather than the power-on value `ff ff`.

MCP4728 address programming is a separate EEPROM operation. Its datasheet
requires a **high-to-low LDAC edge during the low phase of bit 8 of command
byte 2**, held low through command byte 3. On this board the TCA9555 drives
LDAC over the **same I²C wires** as the DAC. A normal TCA9555 output write
cannot occur in the middle of an active MCP4728 I²C command, so the expander
alone cannot generate the required edge. The MCP4728 `RDY/BSY` signal reaches
TCA9555 input P0.7 and TP17; it reports EEPROM activity, but cannot trigger
the address command. On the modified AFE hardware, the direct P0.6-to-LDAC
connection is cut. P0.6 selects the AFE's BSS138, while SPIN pin 6 (PB15)
drives the shared gate line. Each DAC LDAC and each P0.6 selection node needs
its own 5 V pull-up.

After the normal bus devices initialize, the deferred AFE driver checks each
responding expander, selects its DAC through LDAC, discovers its address and
writes EEPROM only if the address differs from the configured address. It
then initializes that AFE. Missing slots are skipped. `afe ID provision`
repeats the discovery and conditional write manually even if that driver
is `NOT READY`; `afe ID provision CURRENT_DAC_ADDR` requires the detected
address to match the supplied value. PA15/PB7 are bit-banged for the read and
write, with a PB15 edge after the 8th SCL pulse of byte 2. The STM32 I²C
controller is recovered after the pins return to I²C mode. The configured
new address must be unused on that U805 channel. Other DACs may stay at the
old address. If a driver failed at boot, power-cycle after a successful
manual command so Zephyr retries initialization. Run the manual command only
while no other I²C transfers are active.

Set `gpio-layout = "v3-1"` in the slot's PEBB overlay when a v3.1 AFE is
installed. The default `v3-0` layout is for v3.0 boards with the LDAC bodge.

Manual control examples:

```text
afe 0 reset
afe 0 gain 3
afe 0 bw 7
afe 0 inputs 1 1
afe 0 dac 0 2500
afe 0 enable 1 1
afe 0 enable 0 0
afe 0 identify
afe 0 identify 5 0
afe 0 identify 0 0
afe 0 stop
pebb term 1
pebb term 0
pebb fan 0 critical
pebb fan 0 ignore
pebb ack
feeder run 1
feeder sample 1234
test feeder
feeder run 0
feeder flush
hb 0 write 100 2048 1024 0
test hb 0
```

`afe ID identify` blinks the selected AFE LED at 2 Hz for three seconds.
Specify 0–20 Hz and an optional duration in milliseconds; duration 0 runs
until `afe ID stop`, while frequency 0 holds the LED on for the duration.
Identification runs in the driver work queue, so the console stays responsive.
`afe ID led 0|1` and `afe ID reset` cancel a running identification.

`afe ... dac` writes only MCP4728 RAM; it does not touch its EEPROM. `hb ...
write` programs the HB 24C02S EEPROM, so use it sparingly and with intended data.
`feeder flush` checkpoints changed data to its wear-levelled EEPROM journal.
`feeder sample` takes a raw measurement code; it does not sample an ADC. Feeder
hours advance only while `feeder run 1` is active.

`pebb scl 1` switches the downstream I²C SCL to PA13. The shipped devicetree
configures PA15, so switching to PA13 can interrupt I²C access; reboot to
restore the startup selection. Keep all analog stages disconnected unless the
board has the required bias and safe external connections.
