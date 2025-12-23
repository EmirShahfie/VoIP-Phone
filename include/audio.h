#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

enum audio_mode {
    AUDIO_MODE_HEADPHONE = 0,
    AUDIO_MODE_SPEAKER   = 1,
};

int audio_init(const struct device *i2s_tx,
               const struct device *i2s_rx,
               const struct i2c_dt_spec *codec_i2c);

int audio_start(void);

/* Called from button/codec module (non-blocking, ISR-safe wrapper) */
void audio_request_toggle_output(void);

/* Called by codec module to actually apply routing (thread context only!) */
int audio_apply_toggle_output(void);

/* Optional: query state */
enum audio_mode audio_get_mode(void);
void audio_set_mode(enum audio_mode m);
