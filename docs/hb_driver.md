# PEBB half-bridge temperature and EEPROM driver

The current HB_board schematic has two Mysentech T117B temperature sensors
(U815 high side and U811 low side) and one XBLW 24C02S EEPROM (U803).
The T117B is not register-compatible with the older TMP117.

## I2C path

MCU I2C1 reaches U805 at `0x70`, channel 4; then the U804 isolator reaches
U814 at `0x71`. U814 channels 2, 3 and 4 reach HB1, HB2 and HB3. The three
branches are electrically separate and can reuse the same addresses.

| HB | PEBB connector | U814 channel | High side | Low side | EEPROM |
| --- | --- | ---: | ---: | ---: | ---: |
| 1 | U829 | 2 | T117B `0x40` | T117B `0x41` | 24C02S `0x50` |
| 2 | U830 | 3 | T117B `0x40` | T117B `0x41` | 24C02S `0x50` |
| 3 | U831 | 4 | T117B `0x40` | T117B `0x41` | 24C02S `0x50` |

On the HB board, connector J827 supplies +5V_PGND and PGND, SCL/SDA, the
sensor ADDR pins and ALERT pins. PEBB straps high side ADDR low and low side
ADDR high. U838 monitors the high/low ALERT pins as pairs 1/0, 3/2 and 5/4.
The selected U814 channel can be checked with the bench `hbdiag` command;
logical bus READY alone does not prove a physical response.

J827 pin 7 is SCL (TP816), pin 8 is SDA (TP815), pin 2 is PGND (TP819),
and pin 1 is +5V_PGND. U803's write-protect input is tied to PGND.
The full connector and memory diagrams are in the internal wiki's
`Shields/PEBB/half-bridge-eeprom.md` page.

## Temperature API

`pebb_hb_get(PEBB_HB_1)` selects a board. `hb_temperature_get()` and
`hb_temperature_get_millicelsius()` select `HB_HIGH_SIDE` or `HB_LOW_SIDE`.
`hb_sensor_device()` exposes the native Zephyr sensor device. The T117B driver
checks ROM bytes `01 16` at registers `0x18`–`0x19`, starts continuous
conversion by writing register `0x04`, and reads temperature from `0x00`–`0x01`.
The signed little-endian code is relative to 25°C with a scale of 1/256°C.
`hb_alert_is_active()` reads the corresponding active-low ALERT input.

## External EEPROM map

`hb_eeprom_device(hb)` returns the board's native 256-byte Zephyr EEPROM.
The operating record occupies `0x00`–`0x07`. A 13-character serial number and
10-character panel key occupy separate 16-byte pages. Text is printable ASCII
without a stored NUL terminator.

| Offset | Content |
| ---: | --- |
| `0x00` | Version, currently 1 |
| `0x01`–`0x03` | Operating hours, unsigned 24-bit little-endian |
| `0x04`–`0x05` low nibble | Maximum voltage, unsigned 12-bit |
| `0x05` high nibble–`0x06` | Maximum current, unsigned 12-bit |
| `0x07` | Flags (bit 0 fault, bit 1 warning) |
| `0x08`–`0x0F` | Unallocated; preserved |
| `0x10`–`0x1C` | Serial number, 13 ASCII bytes |
| `0x1D`–`0x1F` | Unallocated; preserved |
| `0x20`–`0x29` | Panel key, 10 ASCII bytes |
| `0x2A`–`0xFF` | Unallocated; preserved |

`hb_module_data_read()` rejects an unknown version with `-EPROTONOSUPPORT`.
`hb_module_data_write()` writes and verifies those eight bytes, leaving the
rest of the EEPROM untouched. The application must limit write frequency to
respect EEPROM endurance; this driver performs no write at startup.
Records stored in the older TMP117 EEPROM layout are not available on the
new board and require explicit migration if their contents matter.

`hb_serial_number_read/write()` and `hb_panel_key_read/write()` access only
their respective fields. Reads return `-ENODATA` for an erased field and
`-EILSEQ` for invalid ASCII. Writes require the exact character count,
skip an unchanged value, and verify the readback. Callers provide a read
buffer of 14 or 11 bytes respectively, including the added NUL terminator.
The key is stored as plaintext in the physical EEPROM; applications should
avoid printing it in diagnostics. There is no automatic identity write.
