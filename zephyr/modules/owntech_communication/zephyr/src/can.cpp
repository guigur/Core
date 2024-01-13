

#include "can.h"
// #include "GpioApi.h"
#include <zephyr/drivers/gpio.h>

void enable_can()
{
	const struct gpio_dt_spec can_standby_spec = GPIO_DT_SPEC_GET(CAN_STANDBY_DEVICE, gpios);
	gpio_pin_configure_dt(&can_standby_spec, GPIO_OUTPUT_INACTIVE);

    // gpio.configurePin(PB10,OUTPUT);
    // gpio.resetPin(PB10);

}
