# PEBB half-bridge temperature and EEPROM driver

The PEBB shield configures the monitoring devices fitted to the three OwnTech
half-bridge boards. Each HB has two TMP117 temperature sensors. Their four-word
integrated EEPROMs are exposed as normal Zephyr EEPROM devices and through a
small OwnTech API.

## Hardware topology

The power-domain I2C tree is reached through U805 channel 4 and the U804
isolator. U814 is a second TCA9548A at address `0x71`. Zephyr 4.0 initializes
all standard TCA9548A roots with one global priority, which is too early for a
multiplexer behind another mux channel. The `owntech,tca9548a-nested` driver
therefore initializes U814 after the U805 channel and before its own channels.

| Board | U814 channel | HS TMP117 | LS TMP117 | U838 ALERT pins HS/LS |
| --- | ---: | ---: | ---: | ---: |
| HB1 | 2 | `0x48` | `0x49` | 1 / 0 |
| HB2 | 3 | `0x48` | `0x49` | 3 / 2 |
| HB3 | 4 | `0x48` | `0x49` | 5 / 4 |

U838 is a TCA9555-compatible GPIO expander at `0x21` on U814 channel 1. Its
interrupt output is not connected on the PEBB, so ALERT state is polled.

## Application API

Use `pebb_hb_get()` to select one of the three boards and the `hb_*` functions
to select its high-side or low-side device:

```c
#include <errno.h>
#include <hb.h>
#include <pebb_hb.h>

int read_hb1(int32_t *high_mc, int32_t *low_mc)
{
	const struct device *hb = pebb_hb_get(PEBB_HB_1);
	int ret;

	if (!device_is_ready(hb)) {
		return -ENODEV;
	}

	ret = hb_temperature_get_millicelsius(hb, HB_HIGH_SIDE, high_mc);
	if (ret == 0) {
		ret = hb_temperature_get_millicelsius(hb, HB_LOW_SIDE, low_mc);
	}
	return ret;
}
```

The underlying TMP117 driver can return `-EBUSY` if a conversion is not ready;
retry on a later control cycle rather than busy-waiting.

`hb_alert_is_active()` returns 1 when the selected active-low ALERT signal is
asserted. The native sensor and EEPROM devices remain available through
`hb_sensor_device()` and `hb_eeprom_device()` for standard Zephyr APIs.

## TMP117 EEPROM

Use `hb_eeprom_read_word()` and `hb_eeprom_write_word()` for aligned 16-bit
access. Word indices 0, 1 and 3 correspond to user EEPROM1, EEPROM2 and
EEPROM4. Word index 2 is the TMP117 temperature-offset register; the wrapper
rejects access to it with `-EACCES` so calibration cannot be overwritten by a
generic metadata write.

```c
uint16_t board_code;
const struct device *hb = pebb_hb_get(PEBB_HB_2);

/* Read user EEPROM1 from the HB2 high-side TMP117. */
int ret = hb_eeprom_read_word(hb, HB_HIGH_SIDE, 0, &board_code);
```

EEPROM writes are unlocked, performed and read back for verification. They
must only be used for persistent configuration or identification data, never
as periodic logging storage. The driver performs no EEPROM write at startup.
