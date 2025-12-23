#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>

#include <audio.h>
#include <wm8960.h>

#define WM8960_NODE DT_ALIAS(audio0)
#define I2S_TX_NODE DT_ALIAS(i2s_tx)
#define I2S_RX_NODE DT_ALIAS(i2s_rx)

static const struct i2c_dt_spec codec_i2c = I2C_DT_SPEC_GET(WM8960_NODE);
static const struct device *i2s_tx = DEVICE_DT_GET(I2S_TX_NODE);
static const struct device *i2s_rx = DEVICE_DT_GET(I2S_RX_NODE);

int main(void)
{
    int ret;

    if (!device_is_ready(codec_i2c.bus)) {
        printk("I2C bus not ready\n");
        return -1;
    }
    if (!device_is_ready(i2s_tx) || !device_is_ready(i2s_rx)) {
        printk("I2S tx/rx not ready\n");
        return -1;
    }

    ret = wm8960_init(&codec_i2c);
    if (ret) {
        printk("wm8960_init failed: %d\n", ret);
        return ret;
    }

    /* Pick your initial output here */
    ret = wm8960_enable_speakers(&codec_i2c);
    if (ret) {
        printk("enable speakers failed: %d\n", ret);
        return ret;
    }
    audio_set_mode(AUDIO_MODE_SPEAKER);

    ret = wm8960_enable_microphones(&codec_i2c,
                    2,      /* micboost: 0..3 */
                    40,     /* pga_vol: 0..63 */
                    195);   /* adc_dvol: 0..255, 195 ≈ 0 dB */

    if (ret) {
        printk("enable microphones failed: %d\n", ret);
        return ret;
    }

    audio_init(i2s_tx, i2s_rx, &codec_i2c);
    if (ret) {
        printk("audio_init failed: %d\n", ret);
        return ret;
    }

    ret = audio_start();
    if (ret) {
        printk("audio_start failed: %d\n", ret);
        return ret;
    }
    
    printk("System ready. Press button to toggle output.\n");

    /* Nothing else to do; audio thread runs forever */
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
