#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>
#include <wm8960.h>

#define WM8960 DT_ALIAS(audio0)
#define WM8960_BUTTON DT_NODELABEL(user_button)

const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(WM8960);
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(WM8960_BUTTON, gpios);
static struct gpio_callback button_cb_data;

static void audio_switch_work_fn(struct k_work *work);
K_WORK_DEFINE(audio_switch_work, audio_switch_work_fn);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int val = gpio_pin_get_dt(&button);
    if (val > 0) {
        k_work_submit(&audio_switch_work);  /* do I2C later in thread context */
    }
}

static void audio_switch_work_fn(struct k_work *work)
{
    int ret;

    if (get_audio_mode() == AUDIO_MODE_HEADPHONE) {
        ret = wm8960_enable_speakers(&i2c_dev);
        if (ret == 0) {
            set_audio_mode(AUDIO_MODE_SPEAKER);
        } else {
            printk("Failed to switch to speakers (%d)\n", ret);
        }
    } else {
        ret = wm8960_enable_headphones(&i2c_dev);
        if (ret == 0) {
            set_audio_mode(AUDIO_MODE_HEADPHONE);
        } else {
            printk("Failed to switch to headphones (%d)\n", ret);
        }
    }
}

int init_button(void)
{
    int ret;

    if (!device_is_ready(button.port)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0) {
        printk("Error %d: failed to configure button pin %d\n", ret, button.pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on button pin %d\n", ret, button.pin);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    printk("Button configured on pin %d\n", button.pin);

    return 0;
}

int main(void)
{
    int ret;

    if(!device_is_ready(i2c_dev.bus)) {
        printk("I2C bus device not ready\n");
        return -1;
    }

    ret = init_button();
    if (ret) {
        printk("Button initialization failed\n");
        return ret;
    }

    ret = wm8960_setup(&i2c_dev);
    
    if (ret) {
        printk("WM8960 setup failed\n");
        return ret;
    }
    return 0;
}
