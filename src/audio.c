#include <string.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/atomic.h>

#include <audio.h>
#include <wm8960.h>

/* ===== Your audio parameters ===== */
#define SAMPLE_RATE_HZ   44100u
#define TONE_HZ          440u
#define CHANNELS         2u
#define WORD_SIZE_BITS   16u
#define BYTES_PER_SAMPLE (WORD_SIZE_BITS / 8u)
#define BYTES_PER_FRAME  (CHANNELS * BYTES_PER_SAMPLE)

#define BLOCK_SIZE_BYTES 512u
#define BLOCK_COUNT      8u

/* 256-sample sine LUT */
static const int16_t sine256[256] = {
    0,804,1608,2410,3212,4011,4808,5602,6393,7179,7962,8739,9512,10278,11039,11793,
    12539,13279,14010,14732,15446,16151,16846,17530,18204,18868,19519,20159,20787,21403,22005,22594,
    23170,23731,24279,24811,25329,25832,26319,26790,27245,27683,28105,28510,28897,29268,29621,29956,
    30273,30572,30853,31115,31359,31584,31790,31978,32147,32297,32428,32540,32633,32707,32762,32767,
    32762,32707,32633,32540,32428,32297,32147,31978,31790,31584,31359,31115,30853,30572,30273,29956,
    29621,29268,28897,28510,28105,27683,27245,26790,26319,25832,25329,24811,24279,23731,23170,22594,
    22005,21403,20787,20159,19519,18868,18204,17530,16846,16151,15446,14732,14010,13279,12539,11793,
    11039,10278,9512,8739,7962,7179,6393,5602,4808,4011,3212,2410,1608,804,0,-804,
    -1608,-2410,-3212,-4011,-4808,-5602,-6393,-7179,-7962,-8739,-9512,-10278,-11039,-11793,-12539,-13279,
    -14010,-14732,-15446,-16151,-16846,-17530,-18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,-23170,-23731,
    -24279,-24811,-25329,-25832,-26319,-26790,-27245,-27683,-28105,-28510,-28897,-29268,-29621,-29956,-30273,-30572,
    -30853,-31115,-31359,-31584,-31790,-31978,-32147,-32297,-32428,-32540,-32633,-32707,-32762,-32767,-32762,-32707,
    -32633,-32540,-32428,-32297,-32147,-31978,-31790,-31584,-31359,-31115,-30853,-30572,-30273,-29956,-29621,-29268,
    -28897,-28510,-28105,-27683,-27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,-23170,-22594,-22005,-21403,
    -20787,-20159,-19519,-18868,-18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,-12539,-11793,-11039,-10278,
    -9512,-8739,-7962,-7179,-6393,-5602,-4808,-4011,-3212,-2410,-1608,-804
};

K_MEM_SLAB_DEFINE(tx_slab, BLOCK_SIZE_BYTES, BLOCK_COUNT, 4);
K_MEM_SLAB_DEFINE(rx_slab, BLOCK_SIZE_BYTES, BLOCK_COUNT, 4);

#define AUDIO_STACK_SIZE 4096
#define AUDIO_PRIORITY   5

static const struct device *g_i2s_tx;
static const struct device *g_i2s_rx;
static const struct i2c_dt_spec *g_codec;

static struct k_thread audio_thread_data;
K_THREAD_STACK_DEFINE(audio_stack, AUDIO_STACK_SIZE);

static enum audio_mode g_mode = AUDIO_MODE_SPEAKER;

/* Command queue (button posts here) */
enum audio_cmd { AUDIO_CMD_TOGGLE_OUT };
K_MSGQ_DEFINE(audio_cmd_q, sizeof(enum audio_cmd), 8, 4);

static uint32_t phase;
static uint32_t phase_step;
static int16_t amplitude = 6000;

static atomic_t toggle_busy = ATOMIC_INIT(0);

enum audio_mode audio_get_mode(void) { return g_mode; }
void audio_set_mode(enum audio_mode m) { g_mode = m; }

static uintptr_t rx_min, rx_max;
static uintptr_t tx_min, tx_max;

static void compute_slab_bounds(struct k_mem_slab *slab, uintptr_t *min_out, uintptr_t *max_out)
{
    void *p[BLOCK_COUNT];
    uintptr_t mn = UINTPTR_MAX;
    uintptr_t mx = 0;

    /* Allocate all blocks to learn the address range */
    for (int i = 0; i < BLOCK_COUNT; i++) {
        p[i] = NULL;
        if (k_mem_slab_alloc(slab, &p[i], K_NO_WAIT) != 0 || p[i] == NULL) {
            break;
        }
        uintptr_t a = (uintptr_t)p[i];
        if (a < mn) mn = a;
        if (a > mx) mx = a;
    }

    /* Free back what we took */
    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (p[i]) {
            k_mem_slab_free(slab, p[i]);
        }
    }

    *min_out = mn;
    *max_out = mx;
}

static inline bool ptr_in_slab(uintptr_t p, uintptr_t mn, uintptr_t mx)
{
    /* Blocks are contiguous-ish; accept anything in [mn, mx + block_size) */
    return (p >= mn) && (p < (mx + BLOCK_SIZE_BYTES));
}


static void fill_block_stereo_sine(int16_t *dst, size_t frames)
{
    for (size_t i = 0; i < frames; i++) {
        uint8_t idx = (uint8_t)(phase >> 24);
        int32_t s = (int32_t)sine256[idx] * amplitude;
        int16_t sample = (int16_t)(s / 32767);

        dst[2*i + 0] = sample;
        dst[2*i + 1] = sample;
        phase += phase_step;
    }
}

/* Called by wm8960.c (ISR-safe wrapper) */
void audio_request_toggle_output(void)
{
    enum audio_cmd cmd = AUDIO_CMD_TOGGLE_OUT;
    (void)k_msgq_put(&audio_cmd_q, &cmd, K_NO_WAIT);
}

/*
static void reclaim_tx_blocks(void)
{
    void *done = NULL;
    size_t size = 0;

    /* Drain all completed TX blocks 
    while (i2s_read(g_i2s_tx, &done, &size) == 0) {
        k_mem_slab_free(&tx_slab, &done);
        done = NULL;
    }
}
*/

static int i2s_restart_tx(void)
{
    int ret;
    void *blk = NULL;

    /* STOP can return -EIO depending on current state; don't treat as fatal */
    ret = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_STOP);
    if (ret && ret != -EIO) {
        printk("I2S STOP hard fail: %d\n", ret);
        return ret;
    }

    /* DROP is valid in any state except NOT_READY; it resets queues and sets READY */
    ret = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);
    if (ret) {
        printk("I2S DROP failed: %d\n", ret);
        return ret;
    }

    /* IMPORTANT: do NOT call PREPARE here. After DROP we're already READY. */

    /* Prime at least one buffer before START (many drivers require this) */
    ret = k_mem_slab_alloc(&tx_slab, &blk, K_NO_WAIT);
    if (ret) {
        printk("No slab block to prime: %d\n", ret);
        return ret;
    }
    memset(blk, 0, BLOCK_SIZE_BYTES);

    ret = i2s_write(g_i2s_tx, blk, BLOCK_SIZE_BYTES);
    if (ret) {
        k_mem_slab_free(&tx_slab, &blk);
        printk("Prime i2s_write failed: %d\n", ret);
        return ret;
    }

    ret = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret) {
        printk("I2S START failed: %d\n", ret);

        /* If START fails because driver went into ERROR, *then* PREPARE is legal */
        if (ret == -EIO) {
            int r2 = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
            if (r2 == 0) {
                /* re-prime + retry start */
                blk = NULL;
                if (k_mem_slab_alloc(&tx_slab, &blk, K_NO_WAIT) == 0) {
                    memset(blk, 0, BLOCK_SIZE_BYTES);
                    if (i2s_write(g_i2s_tx, blk, BLOCK_SIZE_BYTES) == 0) {
                        ret = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
                    } else {
                        k_mem_slab_free(&tx_slab, blk);
                    }
                }
            }
        }
        return ret;
    }

    return 0;
}

int audio_apply_toggle_output(void)
{
    int ret;

    if (!atomic_cas(&toggle_busy, 0, 1)) {
        return 0;
    }

    /* restart TX cleanly */
    ret = i2s_restart_tx();
    if (ret) goto out;

    /* now change codec path */
    if (g_mode == AUDIO_MODE_HEADPHONE) {
        ret = wm8960_enable_speakers(g_codec);
        if (!ret) g_mode = AUDIO_MODE_SPEAKER;
    } else {
        ret = wm8960_enable_headphones(g_codec);
        if (!ret) g_mode = AUDIO_MODE_HEADPHONE;
    }

    /* if codec change succeeded but you want to be extra safe:
       do another i2s_restart_tx() here */

out:
    atomic_set(&toggle_busy, 0);
    return ret;
}

/*
static void rx_monitor_print(const int16_t *pcm, size_t frames)
{
    /* stereo interleaved: L,R,L,R... 
    int32_t peak = 0;
    int64_t acc = 0;
    size_t samples = frames * 2;

    for (size_t i = 0; i < samples; i++) {
        int32_t s = pcm[i];
        int32_t a = (s < 0) ? -s : s;
        if (a > peak) peak = a;
        acc += (int64_t)s * (int64_t)s;
    }

    /* Cheap RMS (integer); good enough for “is it alive?” 
    int32_t mean = (samples > 0) ? (int32_t)(acc / (int64_t)samples) : 0;

    printk("RX: peak=%ld, mean_sq=%ld\n", (long)peak, (long)mean);
}
*/

static int i2s_init_tx(void)
{
    struct i2s_config cfg = {
        .word_size = WORD_SIZE_BITS,
        .channels = CHANNELS,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .frame_clk_freq = SAMPLE_RATE_HZ,
        .block_size = BLOCK_SIZE_BYTES,
        .mem_slab = &tx_slab,
        .timeout = 1000,
    };
    return i2s_configure(g_i2s_tx, I2S_DIR_TX, &cfg);
}

static int i2s_init_rx(void)
{
    struct i2s_config cfg = {
        .word_size = WORD_SIZE_BITS,
        .channels = CHANNELS,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_RATE_HZ,
        .block_size = BLOCK_SIZE_BYTES,
        .mem_slab = &rx_slab,
        .timeout = 100, /* ms, to “see” failures */
    };
    return i2s_configure(g_i2s_rx, I2S_DIR_RX, &cfg);
}

static int i2s_start_duplex(void)
{
    int ret;
    void *txb = NULL;

    ret = k_mem_slab_alloc(&tx_slab, &txb, K_NO_WAIT);
    if (ret) return ret;
    memset(txb, 0, BLOCK_SIZE_BYTES);

    ret = i2s_write(g_i2s_tx, txb, BLOCK_SIZE_BYTES);
    if (ret) {
        k_mem_slab_free(&tx_slab, txb);
        return ret;
    }

    ret = i2s_trigger(g_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret) return ret;

    ret = i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret) return ret;

    return 0;
}

static int64_t last_recover_ms;

static void i2s_recover(void)
{
    int64_t now = k_uptime_get();

    /* Backoff: don't hammer the driver */
    if ((now - last_recover_ms) < 200) {
        k_msleep(50);
        return;
    }
    last_recover_ms = now;

    (void)i2s_trigger(g_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
    (void)i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_STOP);

    (void)i2s_trigger(g_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);
    (void)i2s_trigger(g_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);

    /* Give DMA/I2S HW time to settle */
    k_msleep(20);

    (void)i2s_start_duplex();
}

static void rx_print_peak(const int16_t *pcm, size_t frames)
{
    int32_t peak = 0;
    size_t n = frames * 2; /* stereo */
    for (size_t i = 0; i < n; i++) {
        int32_t a = pcm[i] < 0 ? -pcm[i] : pcm[i];
        if (a > peak) peak = a;
    }
    printk("RX peak=%ld\n", (long)peak);
}

static int i2s_init_duplex(void)
{
    int ret;

    ret = i2s_init_rx();
    if (ret) {
        printk("I2S RX init failed: %d\n", ret);
        return ret;
    }

    ret = i2s_init_tx();
    if (ret) {
        printk("I2S TX init failed: %d\n", ret);
        return ret;
    }

    phase = 0;
    phase_step = (uint32_t)(((uint64_t)TONE_HZ << 32) / (uint64_t)SAMPLE_RATE_HZ);
    return 0;
}

static void audio_thread(void *a, void *b, void *c)
{
    int ret;

    /* Learn slab pointer ranges (used to validate i2s_read pointers) */
    compute_slab_bounds(&rx_slab, &rx_min, &rx_max);
    compute_slab_bounds(&tx_slab, &tx_min, &tx_max);

    ret = i2s_init_duplex();
    if (ret) return;

    ret = i2s_start_duplex();
    if (ret) return;

    while (1) {
        void *rxb = NULL;
        size_t rxsz = 0;

        ret = i2s_read(g_i2s_rx, &rxb, &rxsz);
        if (ret) {
            /* timeout/transient */
            continue;
        }

        /* HARD GUARDS: if pointer is not from rx_slab, do not touch it */
        uintptr_t ra = (uintptr_t)rxb;
        if (rxb == NULL || rxsz == 0 || rxsz > BLOCK_SIZE_BYTES || !ptr_in_slab(ra, rx_min, rx_max)) {
            /* Don’t free (might not be a slab pointer) */
            i2s_recover();
            continue;
        }

        void *txb = NULL;
        ret = k_mem_slab_alloc(&tx_slab, &txb, K_NO_WAIT);
        if (ret || txb == NULL) {
            /* Safe to free RX (we validated it is from rx_slab) */
            k_mem_slab_free(&rx_slab, rxb);
            continue;
        }

        /* Validate TX block too (paranoia, but prevents freeing wrong pointers later) */
        uintptr_t ta = (uintptr_t)txb;
        if (!ptr_in_slab(ta, tx_min, tx_max)) {
            k_mem_slab_free(&rx_slab, rxb);
            /* Don’t free txb if it’s “weird” (should never happen) */
            i2s_recover();
            continue;
        }

        /* Loopback with attenuation (-12 dB) */
        int16_t *in  = (int16_t *)rxb;
        int16_t *out = (int16_t *)txb;
        size_t samples = rxsz / sizeof(int16_t);

        for (size_t i = 0; i < samples; i++) {
            out[i] = (int16_t)((int32_t)in[i] / 4);
        }

        /* If rxsz < block, pad rest with zeros */
        if (rxsz < BLOCK_SIZE_BYTES) {
            memset((uint8_t *)txb + rxsz, 0, BLOCK_SIZE_BYTES - rxsz);
        }

        k_mem_slab_free(&rx_slab, rxb);

        ret = i2s_write(g_i2s_tx, txb, BLOCK_SIZE_BYTES);
        if (ret) {
            /* Driver didn’t consume -> free back */
            k_mem_slab_free(&tx_slab, txb);
            i2s_recover();
            continue;
        }
    }
}

int audio_init(const struct device *i2s_tx,
               const struct device *i2s_rx,
               const struct i2c_dt_spec *codec_i2c)
{
    g_i2s_tx = i2s_tx;
    g_i2s_rx = i2s_rx;
    g_codec = codec_i2c;
    return 0;
}

int audio_start(void)
{
    k_thread_create(&audio_thread_data, audio_stack, AUDIO_STACK_SIZE,
                    audio_thread, NULL, NULL, NULL,
                    AUDIO_PRIORITY, 0, K_NO_WAIT);
    return 0;
}