/* Interactive PEBB driver bench on the USB console (115200 baud). */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <afe.h>
#include <feeder.h>
#include <hb.h>
#include <pebb.h>
#include <pebb_afe.h>
#include <pebb_hb.h>

#define ANSI_WHITE "\x1b[37m"
#define ANSI_GREEN "\x1b[32m"
#define ANSI_RED "\x1b[31m"
#define ANSI_RESET "\x1b[0m"

static const char *const afe_names[PEBB_AFE_COUNT] = {
    "V_HIGH", "V1_LOW", "V2_LOW", "V3_LOW", "I_HIGH",
    "I1_LOW", "I2_LOW", "I3_LOW", "ANALOG1", "ANALOG2"
};

/* KiCad J822-J831: slot order differs from the logical AFE enum. */
static const uint8_t afe_mux_channels[PEBB_AFE_COUNT] = {
    1, 1, 0, 0, 1, 1, 0, 0, 0, 1
};
static const uint8_t afe_address_indices[PEBB_AFE_COUNT] = {
    1, 0, 4, 1, 4, 2, 0, 3, 2, 3
};
static const char *const afe_ch0_slots[8] = {
    "I2_LOW", "V3_LOW", "ANALOG1", "I3_LOW", "V2_LOW",
    "unexpected", "unexpected", "unexpected"
};
static const char *const afe_ch1_slots[8] = {
    "V1_LOW", "V_HIGH", "I1_LOW", "ANALOG2", "I_HIGH",
    "unexpected", "unexpected", "U839"
};

static const struct i2c_dt_spec pebb_root_mux_i2c =
    I2C_DT_SPEC_GET(DT_NODELABEL(pebb_i2c_mux));

static void recover_i2c(void)
{
    const struct device *bus = pebb_root_mux_i2c.bus;
    if (!device_is_ready(bus)) {
        printk("I2C1 controller: NOT READY\n");
        return;
    }
    printk("Recovering I2C1 on PA15/PB7...\n");
    int ret = i2c_recover_bus(bus);
    printk("I2C1 recovery: %s (%d)\n", ret == 0 ? "OK" : "FAIL", ret);
    if (ret == 0) {
        uint8_t mask = 0;
        ret = i2c_read_dt(&pebb_root_mux_i2c, &mask, sizeof(mask));
        printk("U805 0x70 after recovery: %s (%d)",
               ret == 0 ? "ACK" : "NO", ret);
        if (ret == 0) printk("; selected mask=0x%02x", mask);
        printk("\n");
    }
    printk("Zephyr devices that failed boot initialization remain NOT READY.\n");
}

static int result(const char *name, int ret)
{
    printk("%s: %s (%d)\n", name, ret == 0 ? "OK" : "FAIL", ret);
    return ret != 0;
}

static bool parse(const char *s, unsigned long max, unsigned long *out)
{
    char *end;
    if (!s || !*s || *s == '-') return false;
    errno = 0;
    *out = strtoul(s, &end, 0);
    return errno == 0 && *end == '\0' && *out <= max;
}

static bool dac_readback_plausible(const uint8_t raw[24], uint8_t address)
{
    for (unsigned int ch = 0; ch < 4; ++ch) {
        uint8_t ram = raw[6 * ch];
        uint8_t eeprom = raw[6 * ch + 3];
        if ((ram & 0x80) == 0 || ((ram >> 4) & 3) != ch ||
            ((eeprom >> 4) & 3) != ch ||
            (ram & 7) != (address & 7) ||
            (eeprom & 7) != (address & 7)) return false;
    }
    return true;
}

static const char *probe_status(int ret)
{
    if (ret == 0) return "ACK";
    if (ret == -EIO || ret == -ENXIO) return "NO";
    return "ERROR";
}

static const char *status_color(bool ok)
{
    return ok ? ANSI_GREEN : ANSI_RED;
}

/* Probe components independently, even when an AFE device failed init. */
static void scan_afe_bus(pebb_i2c_bus_id bus_id, const char *name, bool dac_only)
{
    const struct device *bus = pebb_i2c_bus_get(bus_id);
    uint8_t reg = 0x06; /* TCA9555 configuration pair: pointer write + read. */
    uint8_t gpio[2];
    uint8_t dac[24];
    unsigned int found = 0;
    unsigned int bus_errors = 0;

    printk("\n%s - %s addresses:\n", name, dac_only ? "MCP4728" : "TCA9555");
    if (!device_is_ready(bus)) {
        printk("  " ANSI_RED "bus not ready" ANSI_WHITE "\n");
        return;
    }
    for (uint8_t index = 0; index < 8; ++index) {
        uint8_t addr = (dac_only ? 0x60 : 0x20) + index;
        int ret = dac_only ? i2c_read(bus, dac, sizeof(dac), addr) :
            i2c_write_read(bus, addr, &reg, 1, gpio, sizeof(gpio));
        if (ret != 0) {
            if (ret != -EIO && ret != -ENXIO) {
                ++bus_errors;
                printk("  0x%02x: " ANSI_RED "I2C error %d" ANSI_WHITE "\n",
                       addr, ret);
            }
            continue;
        }
        ++found;
        if (dac_only) {
            bool plausible = dac_readback_plausible(dac, addr);
            printk("  0x%02x: " ANSI_GREEN "ACK" ANSI_WHITE "; readback %s%s" ANSI_WHITE "; first bytes %02x %02x %02x\n",
                   addr, status_color(plausible),
                   plausible ? "plausible" : "invalid (collision possible)",
                   dac[0], dac[1], dac[2]);
        } else {
            const char *slot = bus_id == PEBB_I2C_AFE_CH0 ?
                afe_ch0_slots[index] : afe_ch1_slots[index];
            printk("  0x%02x: " ANSI_GREEN "ACK" ANSI_WHITE "; reg6=%02x %02x; expected %s\n",
                   addr, gpio[0], gpio[1], slot);
        }
    }
    if (found == 0)
        printk("  " ANSI_RED "No response in this address range." ANSI_WHITE "\n");
    printk("  Summary: %s%u" ANSI_WHITE " responding address(es), %s%u" ANSI_WHITE " bus error(s).\n",
           status_color(found != 0), found, status_color(bus_errors == 0), bus_errors);
}

static void scan_afe_buses(bool dac_only)
{
    printk(ANSI_WHITE "\n=== %s PHYSICAL ADDRESS SCAN ===\n",
           dac_only ? "DAC" : "GPIO");
    scan_afe_bus(PEBB_I2C_AFE_CH0, "U805 channel 0 (five AFEs)", dac_only);
    scan_afe_bus(PEBB_I2C_AFE_CH1, "U805 channel 1 (five AFEs + U839)", dac_only);
    printk("ACK confirms an address, not a unique chip or its physical slot.\n");
    printk(ANSI_RESET);
}

static int scan(void)
{
    int errors = 0;
    unsigned int mux_ready = 0;
    unsigned int bus_ready = 0;
    unsigned int hb_ready = 0;

    printk(ANSI_WHITE "\n=== ZEPHYR DEVICE STATUS ===\n");
    printk("PEBB board driver: %s%s" ANSI_WHITE "\n",
           status_color(pebb_is_ready()), pebb_is_ready() ? "READY" : "NOT READY");
    errors += !pebb_is_ready();
    for (unsigned int i = 0; i < PEBB_MUX_COUNT; ++i) {
        bool ok = device_is_ready(pebb_mux_get((pebb_mux_id)i));
        mux_ready += ok;
        errors += !ok;
    }
    for (unsigned int i = 0; i < PEBB_I2C_BUS_COUNT; ++i) {
        bool ok = device_is_ready(pebb_i2c_bus_get((pebb_i2c_bus_id)i));
        bus_ready += ok;
        errors += !ok;
    }
    printk("I2C mux drivers: %s%u/%u ready" ANSI_WHITE "; logical buses: %s%u/%u ready" ANSI_WHITE "\n",
           status_color(mux_ready == PEBB_MUX_COUNT), mux_ready, PEBB_MUX_COUNT,
           status_color(bus_ready == PEBB_I2C_BUS_COUNT), bus_ready, PEBB_I2C_BUS_COUNT);
    bool u838_ready = device_is_ready(pebb_alert_gpio_get());
    bool u839_ready = device_is_ready(pebb_control_gpio_get());
    printk("PEBB I/O expanders: U838 alerts %s%s" ANSI_WHITE "; U839 controls %s%s" ANSI_WHITE "\n",
           status_color(u838_ready), u838_ready ? "READY" : "NOT READY",
           status_color(u839_ready), u839_ready ? "READY" : "NOT READY");
    printk("ID AFE slot bus GPIO             DAC              driver\n");
    for (unsigned int i = 0; i < PEBB_AFE_COUNT; ++i) {
        bool ok = device_is_ready(pebb_afe_get((pebb_afe_id)i));
        unsigned int address_index = afe_address_indices[i];
        uint8_t gpio_address = 0x20 + address_index;
        uint8_t dac_address = 0x60 + address_index;
        const struct device *bus = pebb_i2c_bus_get(afe_mux_channels[i] == 0 ?
            PEBB_I2C_AFE_CH0 : PEBB_I2C_AFE_CH1);
        uint8_t reg = 0x06;
        uint8_t gpio[2];
        uint8_t dac[24];
        int gpio_ret = -ENODEV;
        int dac_ret = -ENODEV;
        if (device_is_ready(bus)) {
            gpio_ret = i2c_write_read(bus, gpio_address, &reg, 1,
                                      gpio, sizeof(gpio));
            dac_ret = i2c_read(bus, dac, sizeof(dac), dac_address);
        }
        bool dac_valid = dac_ret == 0 && dac_readback_plausible(dac, dac_address);
        const char *dac_state = dac_ret == 0 && !dac_valid ?
            "ACK_BAD" : probe_status(dac_ret);
        printk("%2u %-7s ch%u 0x%02x %s%-7s" ANSI_WHITE " 0x%02x %s%-8s" ANSI_WHITE " %s%s" ANSI_WHITE "\n",
               i, afe_names[i], afe_mux_channels[i], gpio_address,
               status_color(gpio_ret == 0), probe_status(gpio_ret),
               dac_address, status_color(dac_valid),
               dac_state, status_color(ok), ok ? "READY" : "NOT READY");
        errors += !ok;
    }
    for (unsigned int i = 0; i < PEBB_HB_COUNT; ++i) {
        bool ok = device_is_ready(pebb_hb_get((pebb_hb_id)i));
        hb_ready += ok;
        errors += !ok;
    }
    bool ok = device_is_ready(pebb_feeder_get());
    printk("HB drivers: %s%u/%u ready" ANSI_WHITE "; feeder: %s%s" ANSI_WHITE "\n",
           status_color(hb_ready == PEBB_HB_COUNT), hb_ready, PEBB_HB_COUNT,
           status_color(ok), ok ? "READY" : "NOT READY");
    printk("ACK marks an address response, not a physical slot; an unprovisioned DAC may appear in another row.\n");
    printk("ACK_BAD means the DAC answered with implausible readback.\n");
    scan_afe_buses(false);
    return errors + !ok;
}

static int check_pebb(void)
{
    if (!pebb_is_ready()) return result("PEBB", -ENODEV);
    int errors = 0;
    int v = pebb_fan_faults_poll();
    if (v < 0) errors += result("fan inputs", v);
    else printk("fan fault mask: 0x%x\n", v);
    v = pebb_state_get();
    if (v < 0) errors += result("PEBB state", v);
    else printk("PEBB state: %s\n", v == PEBB_STATE_FAULT ? "FAULT" : "READY");
    v = pebb_fault_mask_get();
    if (v < 0) errors += result("latched faults", v);
    else printk("latched faults: 0x%x\n", v);
    v = pebb_bus_termination_get();
    if (v < 0) errors += result("termination", v);
    else printk("CAN/RS485 termination: %d\n", v);
    v = pebb_i2c_scl_source_get();
    if (v < 0) errors += result("SCL source", v);
    else printk("SCL source: PA%d\n", v ? 13 : 15);
    return errors;
}

static int check_afe(int id)
{
    const struct device *dev = pebb_afe_get((pebb_afe_id)id);
    afe_gpio_state gpio;
    afe_dac_state dac[AFE_DAC_CHANNEL_COUNT];
    printk("AFE %d %s\n", id, afe_names[id]);
    if (!device_is_ready(dev)) return result("ready", -ENODEV);
    int errors = 0;
    int ret = afe_gpio_read(dev, &gpio);
    errors += result("GPIO read", ret);
    if (!ret) printk("  in=%04x out=%04x dir=%04x\n",
                     gpio.inputs, gpio.outputs, gpio.directions);
    ret = afe_dac_read(dev, dac);
    errors += result("DAC read", ret);
    if (!ret) for (unsigned int ch = 0; ch < AFE_DAC_CHANNEL_COUNT; ++ch)
        printk("  DAC %d RAM=%u mV EEPROM=%u mV ready=%d\n", ch,
               dac[ch].value_mv, dac[ch].eeprom_value_mv, dac[ch].ready);
    ret = afe_interrupt_is_pending(dev);
    if (ret >= 0) printk("  INT pending=%d\n", ret);
    else if (ret != -ENOTSUP) errors += result("INT", ret);
    return errors;
}

static int check_hb(int id)
{
    const struct device *dev = pebb_hb_get((pebb_hb_id)id);
    hb_module_data data;
    char serial[HB_SERIAL_NUMBER_LENGTH + 1U];
    char key[HB_PANEL_KEY_LENGTH + 1U];
    printk("HB %d\n", id);
    if (!device_is_ready(dev)) return result("ready", -ENODEV);
    int errors = 0;
    for (int side = HB_HIGH_SIDE; side <= HB_LOW_SIDE; ++side) {
        int32_t mc;
        int ret = hb_temperature_get_millicelsius(dev, (hb_side)side, &mc);
        errors += result(side ? "low temperature" : "high temperature", ret);
        if (!ret) printk("  %s: %d mC\n", side ? "low" : "high", mc);
        ret = hb_alert_is_active(dev, (hb_side)side);
        if (ret < 0) errors += result("T117B alert", ret);
        else printk("  %s alert=%d\n", side ? "low" : "high", ret);
    }
    int ret = hb_module_data_read(dev, &data);
    if (!ret) printk("  EEPROM v%u hours=%u Vmax=%u Imax=%u flags=0x%02x\n",
                     data.version, data.operating_hours, data.max_voltage_code,
                     data.max_current_code, data.flags);
    else if (ret == -EPROTONOSUPPORT) {
        uint8_t raw_version;
        int raw_ret = eeprom_read(hb_eeprom_device(dev), 0,
                                  &raw_version, sizeof(raw_version));
        if (raw_ret == 0 && raw_version == 0xff)
            printk("  EEPROM is blank (version byte 0xff)\n");
        else errors += result("module EEPROM version", raw_ret ? raw_ret : ret);
    }
    else errors += result("module EEPROM", ret);
    ret = hb_serial_number_read(dev, serial);
    if (ret == 0) printk("  serial=%s\n", serial);
    else if (ret == -ENODATA) printk("  serial: blank\n");
    else errors += result("serial", ret);
    ret = hb_panel_key_read(dev, key);
    if (ret == 0) printk("  panel key: programmed\n");
    else if (ret == -ENODATA) printk("  panel key: blank\n");
    else errors += result("panel key", ret);
    return errors;
}

static void hb_diag(void)
{
    static const pebb_i2c_bus_id hb_buses[PEBB_HB_COUNT] = {
        PEBB_I2C_HB_1, PEBB_I2C_HB_2, PEBB_I2C_HB_3
    };
    const struct device *power_bus = pebb_i2c_bus_get(PEBB_I2C_POWER);
    uint8_t mux_mask = 0;
    int ret;

    printk(ANSI_WHITE "\n=== HB I2C DIAGNOSTIC ===\n");
    printk("U805 ch4 -> U804 -> U814 0x71 -> ch2/ch3/ch4 -> HB1/HB2/HB3\n");
    if (device_is_ready(power_bus)) {
        ret = i2c_read(power_bus, &mux_mask, 1, 0x71);
        printk("U814 0x71: %s%s" ANSI_WHITE " (%d)",
               status_color(ret == 0), probe_status(ret), ret);
        if (ret == 0) printk("; selected mask=0x%02x", mux_mask);
        printk("\n");
    } else {
        printk("U805 ch4 logical bus: " ANSI_RED "NOT READY" ANSI_WHITE "\n");
    }

    for (unsigned int i = 0; i < PEBB_HB_COUNT; ++i) {
        const struct device *bus = pebb_i2c_bus_get(hb_buses[i]);
        const struct device *hb = pebb_hb_get((pebb_hb_id)i);
        printk("HB%u (U814 ch%u): bus %s%s" ANSI_WHITE ", driver %s%s" ANSI_WHITE "\n",
               i + 1, i + 2, status_color(device_is_ready(bus)),
               device_is_ready(bus) ? "READY" : "NOT READY",
               status_color(device_is_ready(hb)),
               device_is_ready(hb) ? "READY" : "NOT READY");
        if (!device_is_ready(bus)) continue;
        for (uint8_t addr = 0x40; addr <= 0x43; ++addr) {
            uint8_t id_reg = 0x18;
            uint8_t id[2];
            ret = i2c_write_read(bus, addr, &id_reg, 1, id, sizeof(id));
            if (ret == 0) {
                printk("  T117B address 0x%02x: " ANSI_GREEN "ACK" ANSI_WHITE
                       "; ROM=%02x %02x%s\n", addr, id[0], id[1],
                       id[0] == 0x01 && id[1] == 0x16 ? " (expected)" : " (unexpected)");
            } else {
                printk("  T117B address 0x%02x: " ANSI_RED "%s" ANSI_WHITE " (%d)\n",
                       addr, probe_status(ret), ret);
            }
        }
        uint8_t offset = 0;
        uint8_t first_byte = 0;
        ret = i2c_write_read(bus, 0x50, &offset, 1, &first_byte, 1);
        printk("  24C02S EEPROM 0x50: %s%s" ANSI_WHITE " (%d)",
               status_color(ret == 0), probe_status(ret), ret);
        if (ret == 0) printk("; byte[0]=0x%02x", first_byte);
        printk("\n");
        if (device_is_ready(power_bus)) {
            ret = i2c_read(power_bus, &mux_mask, 1, 0x71);
            printk("  U814 selected mask: %s0x%02x" ANSI_WHITE
                   " (expected 0x%02x; read ret=%d)\n",
                   status_color(ret == 0 && mux_mask == (1U << (i + 2))),
                   ret == 0 ? mux_mask : 0U, 1U << (i + 2), ret);
        }
        for (int side = HB_HIGH_SIDE; side <= HB_LOW_SIDE; ++side) {
            bool sensor = device_is_ready(hb_sensor_device(hb, (hb_side)side));
            printk("  %s (expected 0x%02x): sensor %s%s" ANSI_WHITE "\n",
                   side == HB_HIGH_SIDE ? "HS" : "LS", 0x40 + side,
                   status_color(sensor), sensor ? "READY" : "NOT READY");
        }
        bool eeprom = device_is_ready(hb_eeprom_device(hb));
        printk("  shared EEPROM: %s%s" ANSI_WHITE "\n",
               status_color(eeprom), eeprom ? "READY" : "NOT READY");
    }
    printk("A logical bus can be READY even when its physical devices do not respond.\n");
    printk(ANSI_RESET);
}

static int check_feeder(void)
{
    const struct device *dev = pebb_feeder_get();
    feeder_operating_data data;
    if (!device_is_ready(dev)) return result("feeder", -ENODEV);
    const struct device *eeprom = feeder_eeprom_device(dev);
    if (!device_is_ready(eeprom)) return result("feeder EEPROM", -ENODEV);
    printk("feeder EEPROM size=%u\n", (unsigned)eeprom_get_size(eeprom));
    int ret = feeder_data_get(dev, &data);
    if (!ret) printk("feeder running=%d hours=%u Vvalid=%d Vmin=%u Vmax=%u flags=0x%02x\n",
                     feeder_runtime_is_running(dev), data.operating_hours,
                     data.voltage_valid, data.voltage_min_code,
                     data.voltage_max_code, data.flags);
    return result("feeder data", ret);
}

static void feeder_diag(void)
{
    const struct device *feeder_bus = pebb_i2c_bus_get(PEBB_I2C_FEEDER);
    const struct device *power_bus = pebb_i2c_bus_get(PEBB_I2C_POWER);
    const struct device *power_io_bus = pebb_i2c_bus_get(PEBB_I2C_POWER_IO);
    const struct device *afe_bus_1 = pebb_i2c_bus_get(PEBB_I2C_AFE_CH1);
    uint8_t offset = 0;
    uint8_t bytes[4];
    uint8_t mux_mask;
    uint8_t config_reg = 0x06;
    uint8_t gpio_config[2];
    int ret;

    printk("\n=== FEEDER I2C PATH ===\n");
    if (!i2c_is_ready_dt(&pebb_root_mux_i2c)) {
        printk("U805 root mux 0x70: parent I2C bus NOT READY\n");
    } else {
        ret = i2c_read_dt(&pebb_root_mux_i2c, &mux_mask, 1);
        if (ret == 0)
            printk("U805 root mux 0x70: ACK; selected mask=0x%02x\n", mux_mask);
        else
            printk("U805 root mux 0x70: %s (%d)\n", probe_status(ret), ret);
    }
    if (device_is_ready(afe_bus_1)) {
        ret = i2c_write_read(afe_bus_1, 0x27, &config_reg, 1,
                             gpio_config, sizeof(gpio_config));
        printk("U839 onboard GPIO 0x27 via U805 ch1: %s (%d)\n",
               probe_status(ret), ret);
    }
    printk("Feeder J101 SDA/SCL -> PEBB J811 -> U805 ch2 (DGND).\n");
    if (!device_is_ready(feeder_bus)) {
        printk("U805 ch2 logical bus: NOT READY\n");
    } else {
        ret = i2c_write_read(feeder_bus, 0x50, &offset, 1,
                             bytes, sizeof(bytes));
        if (ret == 0)
            printk("Feeder EEPROM 0x50: ACK; bytes[0..3]=%02x %02x %02x %02x\n",
                   bytes[0], bytes[1], bytes[2], bytes[3]);
        else
            printk("Feeder EEPROM 0x50: %s (%d)\n", probe_status(ret), ret);
    }
    if (i2c_is_ready_dt(&pebb_root_mux_i2c) &&
        i2c_read_dt(&pebb_root_mux_i2c, &mux_mask, 1) == 0)
        printk("U805 selected mask after feeder probe: 0x%02x (expected 0x04)\n",
               mux_mask);

    printk("U805 ch4 -> U804 isolator -> U814 (PGND), separate from feeder EEPROM.\n");
    if (!device_is_ready(power_bus)) {
        printk("U805 ch4 logical bus: NOT READY\n");
    } else {
        ret = i2c_read(power_bus, &mux_mask, 1, 0x71);
        if (ret == 0)
            printk("U814 mux 0x71: ACK; selected mask=0x%02x\n", mux_mask);
        else
            printk("U814 mux 0x71: %s (%d)\n", probe_status(ret), ret);
    }
    if (device_is_ready(power_io_bus)) {
        for (uint8_t addr = 0x20; addr <= 0x21; ++addr) {
            ret = i2c_write_read(power_io_bus, addr, &config_reg, 1,
                                 gpio_config, sizeof(gpio_config));
            printk("U%u GPIO 0x%02x: %s (%d)\n",
                   addr == 0x20 ? 837U : 838U, addr, probe_status(ret), ret);
        }
    }
    printk("Logical READY does not verify that U814 responded electrically.\n");
}

static int check_all(void)
{
    int errors = scan() + check_pebb();
    for (unsigned int i = 0; i < PEBB_AFE_COUNT; ++i) errors += check_afe(i);
    for (unsigned int i = 0; i < PEBB_HB_COUNT; ++i) errors += check_hb(i);
    errors += check_feeder();
    printk("TEST ALL: %d errors\n", errors);
    return errors;
}

static int exercise_afe(unsigned int i)
{
    const struct device *dev = pebb_afe_get((pebb_afe_id)i);
    afe_dac_state before[AFE_DAC_CHANNEL_COUNT];
    afe_dac_state after[AFE_DAC_CHANNEL_COUNT];
    int errors = 0;

    printk("AFE %u controls\n", i);
    if (!device_is_ready(dev)) return result("AFE ready", -ENODEV);
    errors += result("disconnect", afe_set_enabled(dev, false, false));
    errors += result("gain index 0", afe_set_gain(dev, 0));
    errors += result("bandwidth index 7", afe_set_bandwidth(dev, 7));
    errors += result("external inputs", afe_set_inputs(dev, false, false));
    errors += result("LED on", afe_set_led(dev, true));
    errors += result("LED off", afe_set_led(dev, false));
    int ret = afe_dac_read(dev, before);
    errors += result("DAC before", ret);
    if (ret == 0) {
        ret = afe_dac_set_mv(dev, AFE_DAC_SELF_PLUS,
                             before[AFE_DAC_SELF_PLUS].value_mv, false);
        errors += result("DAC volatile write", ret);
        if (ret == 0) {
            ret = afe_dac_read(dev, after);
            errors += result("DAC after", ret);
            if (ret == 0 && after[AFE_DAC_SELF_PLUS].value_mv !=
                            before[AFE_DAC_SELF_PLUS].value_mv)
                errors += result("DAC readback", -EIO);
        }
    }
    return errors;
}

static void exercise_all(void)
{
    int errors = check_all();
    if (pebb_is_ready()) {
        int previous = pebb_bus_termination_get();
        if (previous >= 0) {
            errors += result("termination toggle", pebb_bus_termination_set(!previous));
            errors += result("termination restore", pebb_bus_termination_set(previous));
        }
    }
    for (unsigned int i = 0; i < PEBB_AFE_COUNT; ++i)
        errors += exercise_afe(i);
    printk("EXERCISE ALL: %d errors\n", errors);
}

static void help(void)
{
    printk("scan | dacscan | feederdiag | hbdiag | i2crecover | test all|pebb|feeder|afe ID|all|hb ID|all\n");
    printk("exercise all | exercise afe ID|all (changes AFE controls)\n");
    printk("afe ID reset|gain N|bw N|inputs PLUS MINUS|enable GAIN BW|led 0|1|dac CH MV\n");
    printk("afe ID identify [HZ [DURATION_MS]] | afe ID stop\n");
    printk("afe ID provision CURRENT_DAC_ADDR (experimental EEPROM write; one AFE per branch)\n");
    printk("pebb term 0|1 | pebb scl 0|1 (PA15/PA13) | pebb fan ID ignore|critical | pebb ack\n");
    printk("feeder run 0|1 | feeder sample CODE | feeder flags VALUE | feeder flush\n");
    printk("hb ID write HOURS VMAX IMAX FLAGS (writes HB 24C02S EEPROM)\n");
    printk("IDs start at zero. HB write and feeder checkpoints use EEPROM.\n");
    printk("SCL=1 selects PA13, which requires matching MCU pin configuration.\n");
}

static void command(char *line)
{
    char *a[8] = {};
    char *save = NULL;
    int n = 0;
    unsigned long v[5] = {};
    for (char *p = strtok_r(line, " \t", &save); p && n < 8;
         p = strtok_r(NULL, " \t", &save)) a[n++] = p;
    if (!n) return;
    if (!strcmp(a[0], "help")) { help(); return; }
    if (n == 1 && !strcmp(a[0], "scan")) { scan(); return; }
    if (n == 1 && !strcmp(a[0], "dacscan")) { scan_afe_buses(true); return; }
    if (n == 1 && !strcmp(a[0], "feederdiag")) { feeder_diag(); return; }
    if (n == 1 && !strcmp(a[0], "hbdiag")) { hb_diag(); return; }
    if (n == 1 && !strcmp(a[0], "i2crecover")) { recover_i2c(); return; }
    if (n == 2 && !strcmp(a[0], "exercise") && !strcmp(a[1], "all")) {
        exercise_all(); return;
    }
    if (n == 3 && !strcmp(a[0], "exercise") && !strcmp(a[1], "afe")) {
        if (!strcmp(a[2], "all")) {
            for (unsigned int i = 0; i < PEBB_AFE_COUNT; ++i)
                exercise_afe(i);
            return;
        }
        if (parse(a[2], PEBB_AFE_COUNT - 1, &v[0])) {
            exercise_afe(v[0]); return;
        }
    }
    if (!strcmp(a[0], "test") && n >= 2) {
        if (n == 2 && !strcmp(a[1], "all")) { check_all(); return; }
        if (n == 2 && !strcmp(a[1], "pebb")) { check_pebb(); return; }
        if (n == 2 && !strcmp(a[1], "feeder")) { check_feeder(); return; }
        if (n == 3 && (!strcmp(a[1], "afe") || !strcmp(a[1], "hb"))) {
            bool is_afe = !strcmp(a[1], "afe");
            int limit = is_afe ? PEBB_AFE_COUNT : PEBB_HB_COUNT;
            if (!strcmp(a[2], "all")) {
                for (int i = 0; i < limit; ++i)
                    if (is_afe) check_afe(i); else check_hb(i);
                return;
            }
            if (parse(a[2], limit - 1, &v[0])) {
                if (is_afe) check_afe(v[0]); else check_hb(v[0]);
                return;
            }
        }
    }
    if (!strcmp(a[0], "afe") && n >= 3 && parse(a[1], 9, &v[0])) {
        const struct device *dev = pebb_afe_get((pebb_afe_id)v[0]);
        if (n == 4 && !strcmp(a[2], "provision") &&
            parse(a[3], 0x67, &v[1]) && v[1] >= 0x60) {
            printk("Experimental address write: %s ch%u 0x%02lx -> 0x%02x\n",
                   afe_names[v[0]], afe_mux_channels[v[0]], v[1],
                   0x60 + afe_address_indices[v[0]]);
            int ret = afe_dac_provision_address_experimental(dev, v[1]);
            result("DAC address change verified", ret);
            if (ret == 0) printk("Power-cycle to retry AFE driver initialization.\n");
            scan_afe_buses(true);
            return;
        }
        if (!device_is_ready(dev)) { result("AFE", -ENODEV); return; }
        int ret = -EINVAL;
        if (n == 3 && !strcmp(a[2], "reset")) ret = afe_reset(dev);
        if (n == 3 && !strcmp(a[2], "identify")) ret = afe_identify(dev);
        if (n == 3 && !strcmp(a[2], "stop")) ret = afe_identify_stop(dev);
        if (!strcmp(a[2], "identify") && n >= 4 && n <= 5 &&
            parse(a[3], 20, &v[1]) &&
            (n == 4 || parse(a[4], UINT32_MAX, &v[2])))
            ret = afe_identify_start(dev, v[1], n == 4 ? 3000U : v[2]);
        if (n == 4 && parse(a[3], 7, &v[1])) {
            if (!strcmp(a[2], "gain")) ret = afe_set_gain(dev, v[1]);
            if (!strcmp(a[2], "bw")) ret = afe_set_bandwidth(dev, v[1]);
            if (!strcmp(a[2], "led") && v[1] <= 1) ret = afe_set_led(dev, v[1]);
        }
        if (n == 5 && parse(a[3], 1, &v[1]) && parse(a[4], 1, &v[2])) {
            if (!strcmp(a[2], "inputs")) ret = afe_set_inputs(dev, v[1], v[2]);
            if (!strcmp(a[2], "enable")) ret = afe_set_enabled(dev, v[1], v[2]);
        }
        if (n == 5 && !strcmp(a[2], "dac") &&
            parse(a[3], 3, &v[1]) && parse(a[4], 4095, &v[2]))
            ret = afe_dac_set_mv(dev, (afe_dac_channel)v[1], v[2], false);
        result("AFE control", ret); return;
    }
    if (!strcmp(a[0], "pebb") && n >= 2) {
        int ret = -EINVAL;
        if (n == 2 && !strcmp(a[1], "ack")) ret = pebb_fault_acknowledge();
        if (n == 3 && parse(a[2], 1, &v[0])) {
            if (!strcmp(a[1], "term")) ret = pebb_bus_termination_set(v[0]);
            if (!strcmp(a[1], "scl")) ret = pebb_i2c_scl_source_set((pebb_i2c_scl_source)v[0]);
        }
        if (n == 4 && !strcmp(a[1], "fan") &&
            parse(a[2], PEBB_FAN_COUNT - 1, &v[0])) {
            if (!strcmp(a[3], "ignore"))
                ret = pebb_fan_fault_policy_set((pebb_fan_id)v[0], PEBB_FAN_FAULT_IGNORE);
            if (!strcmp(a[3], "critical"))
                ret = pebb_fan_fault_policy_set((pebb_fan_id)v[0], PEBB_FAN_FAULT_CRITICAL);
        }
        result("PEBB control", ret); return;
    }
    if (!strcmp(a[0], "feeder") && n >= 2) {
        const struct device *dev = pebb_feeder_get();
        if (!device_is_ready(dev)) { result("feeder", -ENODEV); return; }
        int ret = -EINVAL;
        if (n == 2 && !strcmp(a[1], "flush")) ret = feeder_runtime_flush(dev);
        if (n == 3) {
            if (!strcmp(a[1], "run") && parse(a[2], 1, &v[0]))
                ret = feeder_runtime_set_running(dev, v[0]);
            if (!strcmp(a[1], "sample") && parse(a[2], 65535, &v[0]))
                ret = feeder_voltage_observe(dev, v[0]);
            if (!strcmp(a[1], "flags") && parse(a[2], 127, &v[0]))
                ret = feeder_flags_set(dev, v[0]);
        }
        result("feeder control", ret); return;
    }
    if (!strcmp(a[0], "hb") && n == 7 && !strcmp(a[2], "write") &&
        parse(a[1], PEBB_HB_COUNT - 1, &v[0]) &&
        parse(a[3], HB_MODULE_RUNTIME_HOURS_MAX, &v[1]) &&
        parse(a[4], HB_MODULE_MEASUREMENT_MAX, &v[2]) &&
        parse(a[5], HB_MODULE_MEASUREMENT_MAX, &v[3]) &&
        parse(a[6], 255, &v[4])) {
        const struct device *dev = pebb_hb_get((pebb_hb_id)v[0]);
        if (!device_is_ready(dev)) { result("HB", -ENODEV); return; }
        hb_module_data data = {HB_MODULE_DATA_VERSION, (uint32_t)v[1],
                               (uint16_t)v[2], (uint16_t)v[3], (uint8_t)v[4]};
        int ret = hb_module_data_write(dev, &data);
        result("HB EEPROM write", ret);
        if (!ret) check_hb(v[0]);
        return;
    }
    printk("Invalid command. Type help.\n");
}

int main(void)
{
    char line[128];
    size_t used = 0;
    k_msleep(500);
    printk("\n\n========== PEBB DRIVER BENCH READY ==========\n");
    printk("Type help, scan, or dacscan.\n> ");
    while (true) {
        int c = console_getchar();
        if (c < 0) { k_msleep(10); continue; }
        if (c == '\r' || c == '\n') {
            printk("\n");
            line[used] = '\0';
            command(line);
            used = 0;
            printk("> ");
        } else if ((c == '\b' || c == 127) && used > 0) {
            --used; printk("\b \b");
        } else if (c >= 32 && c < 127 && used + 1 < sizeof(line)) {
            line[used++] = (char)c;
            printk("%c", c);
        }
    }
    return 0;
}
