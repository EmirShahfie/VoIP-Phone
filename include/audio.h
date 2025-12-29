#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>

#define SAMPLE_FREQUENCY    48000
#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    (SAMPLE_BIT_WIDTH / 8)
#define NUMBER_OF_CHANNELS  2
#define SAMPLES_PER_BLOCK   (SAMPLE_FREQUENCY / 100)  // 10ms blocks
#define INITIAL_BLOCKS      2
#define TIMEOUT             1000
#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK * NUMBER_OF_CHANNELS)
#define BLOCK_COUNT (INITIAL_BLOCKS + 4)
#define AUDIO_THREAD_STACK_SIZE 2048
#define AUDIO_THREAD_PRIORITY 5

int audio_init(const struct device *i2s_tx,
               const struct device *i2s_rx,
               const struct i2s_config *config_tx,
               const struct i2s_config *config_rx
           );

int audio_prepare(const struct device *i2s_tx);

int audio_start(const struct device *i2s_tx,
                const struct device *i2s_rx);

int audio_stop( const struct device *i2s_tx,
                const struct device *i2s_rx);

void start_audio_thread(const struct device *i2s_tx,
                        const struct device *i2s_rx,
                        const struct i2s_config *config_tx,
                        const struct i2s_config *config_rx);