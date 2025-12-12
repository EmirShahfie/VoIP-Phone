#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>
#include <wm8960.h>

#define WM8960 DT_ALIAS(audio0)

int main(void)
{
    const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(WM8960);

    if(!device_is_ready(i2c_dev.bus)) {
        printk("I2C bus device not ready\n");
        return -1;
    }

    int ret = wm8960_setup(&i2c_dev);
    
    if (ret) {
        printk("WM8960 setup failed\n");
        return ret;
    }
    return 0;
}
