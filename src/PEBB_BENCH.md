# PEBB driver bench

Build and flash the `PEBB` branch with the `USB` PlatformIO environment. Open
the USB serial console at 115200 baud, press Enter, and type `help`.

The bench replaces the former LED-blink `src/main.cpp`. Commands use zero-based
IDs: AFEs 0–9 follow `pebb_afe_id`, and HB modules 0–2 follow `pebb_hb_id`.

| Command | What it checks or changes |
| --- | --- |
| `scan` | Zephyr readiness of the PEBB, 2 I²C muxes, 8 logical buses, 10 AFEs, 3 HB modules and feeder |
| `test all` | Read-only checks of PEBB controls/fan status, every AFE GPIO/DAC, every HB temperature/alert/module record, and feeder EEPROM/data |
| `test pebb`, `test feeder` | Read-only checks of one subsystem |
| `test afe 0`, `test afe all` | Read one or all AFE GPIO registers, DAC RAM/EEPROM and INT status |
| `test hb 0`, `test hb all` | Read one or all HB temperatures, alerts and EEPROM records |
| `exercise afe 0`, `exercise afe all` | Disconnect stages, set gain index 0 and bandwidth index 7, select external inputs, toggle LED, rewrite the current DAC A RAM value and read it back |
| `exercise all` | Run `test all`, toggle/restore CAN/RS485 termination and exercise all AFEs |

The test does not claim to validate analog gain or temperature accuracy: those
need external stimulus or a reference instrument. An erased HB EEPROM is shown
as blank; an unexpected record version or I²C failure is reported as an error.

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
