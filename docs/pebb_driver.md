# PEBB board driver

The PEBB shield describes the board's I2C tree. The `owntech_pebb_driver`
module owns its topology, the nested TCA9548A driver, and the group APIs for
the ten AFEs and three half-bridge boards. The AFE, HB and feeder modules own
the devices on those buses.

## I2C topology

| Device / bus | Parent | Address or channel |
| --- | --- | --- |
| U805 root TCA9548A | MCU I2C1 | `0x70` |
| Eight main AFEs | U805 | channel 0 |
| SIN/COS AFEs | U805 | channel 1 |
| U814 power TCA9548A | U805 channel 4 via U804 isolator | `0x71` |
| Low-voltage feeder | U814 | channel 0 |
| U838 alert GPIO expander | U814 | channel 1, address `0x21` |
| HB1, HB2, HB3 | U814 | channels 2, 3, 4 |

The U805 mux uses Zephyr's standard `ti,tca9548a` driver. U814 uses
`owntech,tca9548a-nested` because its parent mux channel must initialize
first. Applications use the child I2C bus devices directly. Zephyr selects the
channels during each I2C operation; applications do not write mux masks.

## Board API

Include `<pebb.h>`. `pebb_is_ready()` checks the on-board muxes, channel
devices, and U838/U839 GPIO expanders. It does not require external AFE, feeder, or HB
cards to be present. The getter functions return Zephyr device pointers,
or `NULL` for an invalid enum value.

```c
#include <pebb.h>
#include <zephyr/drivers/i2c.h>

if (pebb_is_ready()) {
	const struct device *bus = pebb_i2c_bus_get(PEBB_I2C_HB_1);
	/* Use bus with standard Zephyr I2C APIs. */
}
```

`pebb_mux_get(PEBB_MUX_ROOT)` and `pebb_mux_get(PEBB_MUX_POWER)` expose the
two mux devices for diagnostics. `pebb_alert_gpio_get()` returns U838 and
`pebb_control_gpio_get()` returns U839. The
`pebb_feeder_get()` returns the low-voltage feeder device, whose readiness
depends on the external feeder board. The functional group APIs remain
`<pebb_afe.h>` and `<pebb_hb.h>`; see
[AFE driver](afe_driver.md) and [HB driver](hb_driver.md).

Enable the shield with `board_shield = pebb`. Its `pebb.conf` enables
`CONFIG_OWNTECH_PEBB_DRIVER`, the mux driver, and the component drivers.

## Fans, bus termination and SCL selection

`pebb_fan_fault_get(PEBB_FAN_1)` through `PEBB_FAN_3` return 1 when the
corresponding tach watchdog reports a fault, 0 when it sees tach activity,
or a negative errno. The three active-low inputs are U838 pins 7, 14 and 15.
Allow the fan time to start before treating an initial fault as persistent.

Each fan starts with `PEBB_FAN_FAULT_IGNORE`. After a fan has started, call
`pebb_fan_fault_policy_set(fan, PEBB_FAN_FAULT_CRITICAL)` to make its fault
change the board state from `PEBB_STATE_READY` to `PEBB_STATE_FAULT`. U838's
interrupt is not connected, so the driver polls every
`CONFIG_OWNTECH_PEBB_FAN_POLL_MS` milliseconds (250 ms by default). The fault
state and its bit mask remain latched until `pebb_fault_acknowledge()` succeeds.
Acknowledgment returns `-EBUSY` while a monitored fault is still present.

`pebb_state_callback_set(callback, user_data)` reports the transition to
`PEBB_STATE_FAULT` and the return to `PEBB_STATE_READY`. Periodic polls call
it from the Zephyr system work queue; explicit polls and acknowledgment call
it from their caller's context. Keep it short and submit longer actions to an
application task. A critical read failure also latches `PEBB_FAULT_FAN_IO`.
The application decides whether a fault state stops PWM/power, changes its
own state machine or raises an alarm. Register the callback before enabling
critical monitoring. Setting a policy back to IGNORE does not erase a
previously latched fault; acknowledge it once the physical alert has cleared.

`pebb_bus_termination_set(true)` enables both the CAN and RS485 120-ohm
terminations; `false` disables both. They share U839 pin 0 (`TERM120`) and
cannot be controlled independently. `pebb_bus_termination_get()` returns 1
when enabled, 0 when disabled, or a negative errno. Initialization leaves
both terminations disabled.

`pebb_i2c_scl_source_set(PEBB_I2C_SCL_PA15)` selects the default PA15 input
of U726; `PEBB_I2C_SCL_PA13` selects PA13. The state can be read with
`pebb_i2c_scl_source_get()`. The PEBB shield configures Zephyr I²C1 on PA15.
Switching to PA13 disconnects that controller's SCL from U805, so further
I²C access through the current Zephyr configuration, including the call to
switch back, may fail. Use the PA13 setting only as part of an intentional
controller handoff with a recovery path.
