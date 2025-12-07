#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#define WM8960_NODE DT_NODELABEL(wm8960)

static const struct i2c_dt_spec wm8960_i2c = I2C_DT_SPEC_GET(WM8960_NODE);

static int wm8960_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (reg << 1) | ((val >> 8) & 0x01);
    buf[1] = val & 0xFF;

    return i2c_write_dt(&wm8960_i2c, buf, sizeof(buf));
}

int main(void)
{
    printk("wm8960_i2c.bus name: %s\n", wm8960_i2c.bus->name);
    printk("wm8960_i2c.addr: 0x%x\n", wm8960_i2c.addr);

    if (!device_is_ready(wm8960_i2c.bus)) {
        printk("I2C bus device not ready\n");
        return -1;
    }

    printk("I2C bus is ready, trying WM8960 reset...\n");

    int ret = wm8960_write_reg(0x0F, 0x000);  // reset reg
    if (ret) {
        printk("Failed to write to WM8960 register, ret = %d\n", ret);
        return -1;
    } else {
        printk("WM8960 reset successfully\n");
    }

    while (1) {
        k_sleep(K_MSEC(1000));
    }
    return 0;
}