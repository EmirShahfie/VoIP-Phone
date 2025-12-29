#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>

#include <audio.h>
#include <wm8960.h>
#include <network.h>

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

    wifi_init();

    ret = wifi_connect();
    if (ret) {
        printk("WiFi connect failed: %d\n", ret);
        return ret;
    }

    ret = udp_init();
    if (ret) {
        printk("UDP init failed: %d\n", ret);
        return ret;
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

    ret = wm8960_enable_microphones(&codec_i2c,
                    2,      /* micboost: 0..3 */
                    40,     /* pga_vol: 0..63 */
                    195);   /* adc_dvol: 0..255, 195 ≈ 0 dB */

    if (ret) {
        printk("enable microphones failed: %d\n", ret);
        return ret;
    }
    
    struct i2s_config i2s_cfg = {
        .word_size = SAMPLE_BIT_WIDTH,
        .channels = NUMBER_OF_CHANNELS,
        .format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_BIT_CLK_INV,
        .options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
        .frame_clk_freq = SAMPLE_FREQUENCY,
        .mem_slab = NULL, // Will be set in audio.c
        .block_size = BLOCK_SIZE,
        .timeout = TIMEOUT,
    };

    start_audio_thread(i2s_tx, i2s_rx, &i2s_cfg, &i2s_cfg);

    /* Nothing else to do; audio thread runs forever */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
