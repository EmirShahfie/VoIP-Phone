#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/*
 * Use the default I2C0 device from your ESP32-S3 board.
 * This resolves to something like "i2c@60013000".
 */
#define I2C_DEV DT_NODELABEL(i2c0)

void main(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV);

    if (!device_is_ready(i2c_dev)) {
        printk("I2C0 device is NOT ready!\n");
        return;
    }

    printk("I2C0 device is ready: %s\n", i2c_dev->name);
    printk("Starting I2C scan...\n");

    uint8_t dummy = 0x00;
    uint8_t addr;

    for (addr = 0x03; addr <= 0x77; addr++) {

        /* Try writing a single dummy byte */
        int ret = i2c_write(i2c_dev, &dummy, 1, addr);

        if (ret == 0) {
            printk("FOUND device at 0x%02X\n", addr);
        } else if (ret != -EIO && ret != -ENXIO) {
            /* Only print unexpected errors */
            printk("Address 0x%02X returned error %d\n", addr, ret);
        }

        k_msleep(5);
    }

    printk("I2C scan finished.\n");
}
