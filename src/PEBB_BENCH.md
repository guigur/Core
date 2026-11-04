# PEBB driver bench

Build and flash the `PEBB` branch with the `USB` PlatformIO environment. Open
the USB serial console at 115200 baud, press Enter, and type `help`.

The bench replaces the former LED-blink `src/main.cpp`. Commands use zero-based
IDs: AFEs 0–9 follow `pebb_afe_id`, and HB modules 0–2 follow `pebb_hb_id`.

| Command | What it checks or changes |
| --- | --- |
| `scan` | Named Zephyr device status plus TCA9555 probes at `0x20`–`0x27` on both AFE I²C branches |
| `dacscan` | Separate MCP4728 probes at `0x60`–`0x67` on both branches, even if AFE drivers failed initialization |
| `afe 2 provision 0x60` | Experimental MCP4728 EEPROM address attempt for V2_LOW, from `0x60` to configured `0x64`; requires V2_LOW alone on U805 channel 0 |
| `test all` | Read-only checks of PEBB controls/fan status, every AFE GPIO/DAC, every HB temperature/alert/module record, and feeder EEPROM/data |
| `test pebb`, `test feeder` | Read-only checks of one subsystem |
| `test afe 0`, `test afe all` | Read one or all AFE GPIO registers, DAC RAM/EEPROM and INT status |
| `test hb 0`, `test hb all` | Read one or all HB temperatures, alerts and EEPROM records |
| `exercise afe 0`, `exercise afe all` | Disconnect stages, set gain index 0 and bandwidth index 7, select external inputs, toggle LED, rewrite the current DAC A RAM value and read it back |
| `exercise all` | Run `test all`, toggle/restore CAN/RS485 termination and exercise all AFEs |

The test does not claim to validate analog gain or temperature accuracy: those
need external stimulus or a reference instrument. An erased HB EEPROM is shown
as blank; an unexpected record version or I²C failure is reported as an error.

## Diagnosing AFE address conflicts

The updated PEBB schematic places five AFE connectors on each U805 channel.
The connector A0/A1/A2 straps set the TCA9555 address as follows:

| U805 channel | Slot (connector) | TCA9555 | Expected MCP4728 |
| --- | --- | --- | --- |
| 0 | I2_LOW (J823) | `0x20` | `0x60` |
| 0 | V3_LOW (J830) | `0x21` | `0x61` |
| 0 | SIN (J825) | `0x22` | `0x62` |
| 0 | I3_LOW (J824) | `0x23` | `0x63` |
| 0 | V2_LOW (J829) | `0x24` | `0x64` |
| 1 | V1_LOW (J828) | `0x20` | `0x60` |
| 1 | V_HIGH (J827) | `0x21` | `0x61` |
| 1 | I1_LOW (J831) | `0x22` | `0x62` |
| 1 | COS (J826) | `0x23` | `0x63` |
| 1 | I_HIGH (J822) | `0x24` | `0x64` |

U839 is also on channel 1 at `0x27`. The MCP4728 addresses in the last column
are the **software configuration**, not connector straps: each DAC stores its
address in its own EEPROM and normally starts at `0x60`. Each fitted board
must have its DAC programmed to the expected address before two boards with
the same DAC address share a channel.

`scan` and `dacscan` list **actual address responses on each mux channel**.
The logical `READY` line shows which configured Zephyr device initialized; it
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
the address command. The AFE schematic exposes LDAC at TP18. A commissioning
fixture can drive TP18 directly from an MCU GPIO while sending a bit-timed
MCP4728 command, with the TCA9555 P0.6 configured as input to avoid contention.
Do this with one unassigned DAC at a time, then verify its new address before
fitting another board on the same I²C branch.

For diagnostics, `afe ID provision CURRENT_DAC_ADDR` makes one experimental
address-write attempt and checks both the old and configured new address. It
works even when the AFE driver is `NOT READY`. Disconnect other AFEs from the
same U805 channel first: multiple DACs can share the source address and a
readback cannot prove uniqueness. The TCA9555 can pulse LDAC only **before**
the DAC transaction, so this attempt does not meet the MCP4728 timing rule
and is expected to fail on a conforming chip. A reported success requires the
new address to respond and the old address to stop responding; power-cycle
after success. The command does not claim success from an ACK alone.

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
write` programs the TMP117 EEPROM, so use it sparingly and with intended data.
`feeder flush` checkpoints changed data to its wear-levelled EEPROM journal.
`feeder sample` takes a raw measurement code; it does not sample an ADC. Feeder
hours advance only while `feeder run 1` is active.

`pebb scl 1` switches the downstream I²C SCL to PA13. The shipped devicetree
configures PA15, so switching to PA13 can interrupt I²C access; reboot to
restore the startup selection. Keep all analog stages disconnected unless the
board has the required bias and safe external connections.
