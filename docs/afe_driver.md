# OwnTech AFE v3 Zephyr driver

The driver supports any number of AFE instances. An AFE can be connected
directly to an I2C controller or placed below a Zephyr I2C mux channel. The AFE
API is identical in both cases because the native Zephyr TCA9548A driver selects
and locks the correct channel for every transaction.

## Configuration

Enable the driver in `src/app.conf`:

```conf
CONFIG_OWNTECH_AFE_DRIVER=y
# Required when a TCA9548A node is enabled with the Zephyr 4.0 driver shipped
# with this project:
CONFIG_I2C_TCA954X_ROOT_INIT_PRIO=50
CONFIG_I2C_TCA954X_CHANNEL_INIT_PRIO=51
```

The following example describes one direct AFE and two AFEs with identical
addresses behind channels 0 and 3 of a TCA9548A. Adapt the controller, pins,
addresses and interrupt GPIOs to the hardware. `clock-frequency` is kept at
100 kHz for initial validation.

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/dt-bindings/i2c/i2c.h>

/* Illustrative pins only: choose pins not used by your power stage. */
&i2c3 {
	compatible = "st,stm32-i2c-v2";
	pinctrl-0 = <&i2c3_scl_pc8 &i2c3_sda_pc9>;
	pinctrl-names = "default";
	clock-frequency = <I2C_BITRATE_STANDARD>;
	status = "okay";

	/* A direct AFE must not collide with devices exposed by an open mux. */
	afe_direct: afe@21 {
		compatible = "owntech,afe-v3";
		reg = <0x21>;            /* TCA9555 address */
		dac-address = <0x61>;    /* programmed MCP4728 address */
		int-gpios = <&gpioc 5 GPIO_ACTIVE_LOW>;
	};

	i2c_mux: tca9548a@70 {
		compatible = "ti,tca9548a";
		reg = <0x70>;
		#address-cells = <1>;
		#size-cells = <0>;
		status = "okay";

		mux_i2c_0: i2c@0 {
			compatible = "ti,tca9548a-channel";
			reg = <0>;
			#address-cells = <1>;
			#size-cells = <0>;

			afe_0: afe@20 {
				compatible = "owntech,afe-v3";
				reg = <0x20>;
				dac-address = <0x60>;
				int-gpios = <&gpioc 6 GPIO_ACTIVE_LOW>;
			};
		};

		mux_i2c_3: i2c@3 {
			compatible = "ti,tca9548a-channel";
			reg = <3>;
			#address-cells = <1>;
			#size-cells = <0>;

			afe_3: afe@20 {
				compatible = "owntech,afe-v3";
				reg = <0x20>;
				dac-address = <0x60>;
				int-gpios = <&gpioc 7 GPIO_ACTIVE_LOW>;
			};
		};
	};
};
```

The TCA9548A address is `0x70 + A2:A0`. Each `i2c@N` child selects channel N.
Nested I2C muxes also work: place another TCA9548A node below a channel node.

The AFE TCA9555 address is `0x20 + A2:A0`. If these address pins are driven by
the MCU rather than strapped, add three GPIOs in A0, A1, A2 order. The driver
sets them before its first transaction and checks that `reg` is in `0x20..0x27`:

```dts
	afe_addressed: afe@25 {
		compatible = "owntech,afe-v3";
		reg = <0x25>;
		dac-address = <0x60>;
		address-gpios = <&gpioa 0 (GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN)>,
				  <&gpioa 1 (GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN)>,
				  <&gpioa 2 (GPIO_ACTIVE_HIGH | GPIO_OPEN_DRAIN)>;
	};
```

The AFE logic is powered at 5 V. A 3.3 V push-pull high is not a guaranteed
logic high for a 5 V TCA9555. Strap A0/A1/A2 when possible; otherwise use
5 V tolerant open-drain MCU pins with external pull-ups to 5 V, or a level
translator. Apply the same voltage-domain checks to SCL, SDA and `~INT`.

The MCP4728 address is independent. It is stored in MCP4728 EEPROM and is only
declared with `dac-address`; this driver deliberately never changes it and never
writes any DAC EEPROM value.

## Application API

Each node creates a normal Zephyr device. Obtain every AFE independently by its
node label:

```c
#include <errno.h>
#include <zephyr/device.h>
#include <afe.h>

static const struct device *const afe0 = DEVICE_DT_GET(DT_NODELABEL(afe_0));
static const struct device *const afe3 = DEVICE_DT_GET(DT_NODELABEL(afe_3));

int configure_afes(void)
{
	const uint16_t self_test_mv[4] = {2600, 2400, 2500, 3976};
	int ret;

	if (!device_is_ready(afe0) || !device_is_ready(afe3)) {
		return -ENODEV;
	}

	ret = afe_dac_set_all_mv(afe0, self_test_mv);
	if (ret == 0) {
		ret = afe_set_inputs(afe0, true, true);
	}
	if (ret == 0) {
		ret = afe_set_gain(afe0, 4); /* nominal x1 */
	}
	if (ret == 0) {
		ret = afe_set_bandwidth(afe0, 7);
	}
	if (ret == 0) {
		ret = afe_set_enabled(afe0, true, true);
	}

	return ret;
}
```

At boot, every enabled AFE instance is initialized with x1, BW7, external
inputs and both analog stages disconnected. The default DAC RAM values are
2500, 2500, 2500 and 3976 mV. These can be overridden per node with
`startup-gain`, `startup-bandwidth` and `startup-dac-millivolts`.

`afe_interrupt_is_pending()` reads the optional active-low `int-gpios` line.
`afe_interrupt_acknowledge()` reads both TCA9555 input ports, which clears the
latched interrupt condition. Do not perform this I2C acknowledgement in a GPIO
ISR; defer it to a thread or work item.

## PEBB shield: ten AFE modules

Select the `pebb` shield to configure the ten AFE v3 modules present on the
PEBB schematic:

```ini
board = spin
board_shield = pebb
```

The shield uses I2C1 on PA15 (SCL) and PB7 (SDA), then U805 (TCA9548A at
`0x70`) to split the bus. TIM4 is disabled because its encoder channel also
uses PB7 on the SPIN board.

| Functional ID | Mux channel | TCA9555 | MCP4728 |
| --- | ---: | ---: | ---: |
| `PEBB_AFE_V_HIGH` | 0 | `0x20` | `0x60` |
| `PEBB_AFE_V1_LOW` | 0 | `0x21` | `0x61` |
| `PEBB_AFE_V2_LOW` | 0 | `0x22` | `0x62` |
| `PEBB_AFE_V3_LOW` | 0 | `0x23` | `0x63` |
| `PEBB_AFE_I_HIGH` | 0 | `0x24` | `0x64` |
| `PEBB_AFE_I1_LOW` | 0 | `0x25` | `0x65` |
| `PEBB_AFE_I2_LOW` | 0 | `0x26` | `0x66` |
| `PEBB_AFE_I3_LOW` | 0 | `0x27` | `0x67` |
| `PEBB_AFE_SIN` | 1 | `0x20` | `0x60` |
| `PEBB_AFE_COS` | 1 | `0x21` | `0x61` |

The schematic leaves the ten AFE interrupt outputs unconnected, so the PEBB
nodes do not declare `int-gpios`. The group helper provides stable functional
names and best-effort bulk operations:

```c
#include <errno.h>
#include <afe.h>
#include <pebb_afe.h>

if (!pebb_afe_all_ready()) {
	return -ENODEV;
}

const struct device *v_high = pebb_afe_get(PEBB_AFE_V_HIGH);
afe_set_gain(v_high, 3);

/* Safe startup keeps all analog paths disconnected. */
pebb_afe_set_enabled_all(true, true);
```
