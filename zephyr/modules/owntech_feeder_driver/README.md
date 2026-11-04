# OwnTech low-voltage feeder driver

This module exposes the feeder's XBLW 24C02S EEPROM and maintains a persistent
operating record. On the PEBB, the EEPROM is reached through U805 channel 4 and
U814 channel 0 at I2C address `0x50`.

## EEPROM map

The 256-byte EEPROM is divided into sixteen rotating 16-byte records. No bytes
are available as an unstructured application EEPROM region. All multibyte
fields are little-endian.

| Record byte(s) | Size | Field | Meaning |
| --- | ---: | --- | --- |
| 0 | 1 | Type | `0x01` = low-voltage feeder; `0x02` = high-voltage feeder (reserved, rejected by this driver) |
| 1 | 1 | Version | `0x01` |
| 2..3 | 2 | Sequence | Incremented per successful checkpoint; wraps modulo 2^16 |
| 4..6 | 3 | Operating hours | Completed whole hours, unsigned 24-bit |
| 7..8 | 2 | Minimum voltage code | Unsigned raw measurement code |
| 9..10 | 2 | Maximum voltage code | Unsigned raw measurement code |
| 11 | 1 | Flags | Bit 0 fault; bit 1 warning; bit 7 indicates valid voltage extrema |
| 12..15 | 4 | CRC-32 | `crc32_ieee()` over bytes 0..11 |

Record bases are `0x00`, `0x10`, `0x20`, ... `0xF0`. The driver scans the
records at boot and loads the valid record with the newest sequence. The
low-voltage driver also reads legacy records beginning with `46 32` (`F2`) so
existing operating hours are retained; the next checkpoint uses the new
`01 01` header. A torn or corrupted record fails the CRC and is ignored.

## Persistence and API

The default checkpoint interval is 60 accumulated operating minutes. The
record is checkpointed periodically while the feeder is marked running, and
pages rotate so a given EEPROM page is normally written once per sixteen
operating hours. The hour field has one-hour resolution; an unexpected power
loss can lose runtime and voltage extrema since the last checkpoint. Call
`feeder_runtime_flush()` before a controlled shutdown to force a checkpoint.

The application marks the feeder active with `feeder_runtime_set_running()`
and reports voltage samples with `feeder_voltage_observe()`. The driver tracks
minimum and maximum raw voltage codes in RAM. Voltage conversion to volts is
application-specific. Flags are set with `feeder_flags_set()` and read through
`feeder_data_get()`.

```c
struct feeder_operating_data record;

feeder_runtime_set_running(feeder, true);
feeder_voltage_observe(feeder, voltage_adc_code);
feeder_flags_set(feeder, FEEDER_FLAG_WARNING);
feeder_data_get(feeder, &record);
/* Before a controlled shutdown: */
feeder_runtime_set_running(feeder, false);
feeder_runtime_flush(feeder);
```

The `voltage_min_code` and `voltage_max_code` members are raw 16-bit codes.
`voltage_valid` is false until the first call to `feeder_voltage_observe()`.
`operating_hours` saturates at `0xFFFFFF`.
