#include <string.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/atomic.h>

#include <audio.h>
#include <wm8960.h>
#include <network.h>
#include <packet.h>

K_MEM_SLAB_DEFINE_STATIC(tx_mem_slab,BLOCK_SIZE,BLOCK_COUNT,4);
K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab,BLOCK_SIZE,BLOCK_COUNT,4);

K_THREAD_STACK_DEFINE(audio_thread_stack, AUDIO_THREAD_STACK_SIZE);
struct k_thread audio_thread_data;

static const struct device *i2s_tx_device;
static const struct device *i2s_rx_device;

int audio_prepare(const struct device *i2s_tx)
{
    void *block;
    int ret;

    for (int i = 0; i < INITIAL_BLOCKS; i++) {
        ret = k_mem_slab_alloc(&tx_mem_slab, &block, K_NO_WAIT);
        if (ret < 0) {
            printk("Failed to allocate TX block: %d\n", ret);
            return ret;
        }

        // Fill block with audio data (e.g., from a buffer or generate silence)
        memset(block, 0, BLOCK_SIZE); // Example: fill with silence

        ret = i2s_write(i2s_tx, block, BLOCK_SIZE);
        if (ret < 0) {
            printk("Failed to queue TX block: %d\n", ret);
            k_mem_slab_free(&tx_mem_slab, block);
            return ret;
        }
    }
    return 0;
}

int audio_init( const struct device *i2s_tx,
                const struct device *i2s_rx,
                const struct i2s_config *config_tx,
                const struct i2s_config *config_rx
            )
{
    int ret;
    
    // Configure i2s devices for RX and TX
    ret = i2s_configure(i2s_rx, I2S_DIR_RX, config_rx);
	if (ret < 0) {
		printk("Failed to configure RX stream: %d\n", ret);
		return ret;
	}

	ret = i2s_configure(i2s_tx, I2S_DIR_TX, config_tx);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return ret;
	}

    ret = audio_prepare(i2s_tx);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int audio_start(const struct device *i2s_tx,
                const struct device *i2s_rx)
{   
    int ret;

    ret = i2s_trigger(i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start RX stream: %d\n", ret);
        return ret;
    }

    ret = i2s_trigger(i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start TX stream: %d\n", ret);
        return ret;
    }
    return 0;
}

int audio_stop( const struct device *i2s_tx,
                const struct device *i2s_rx)
{
    int ret;

    ret = i2s_trigger(i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);
    if (ret < 0) {
        printk("Failed to stop TX stream: %d\n", ret);
        return ret;
    }
    ret = i2s_trigger(i2s_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);
    if (ret < 0) {
        printk("Failed to stop RX stream: %d\n", ret);
        return ret;
    }
    return 0;
}

static void audio_loopback(void){
    int ret;
    while(1){
        /***************** RX PROCESS *****************/
        void *rx_block;
        size_t rx_size;

        // Read audio data from RX into rx memory block
        ret = i2s_read(i2s_rx_device, &rx_block, &rx_size);
        if (ret < 0) {
            printk("Failed to read RX block: %d\n", ret);
            continue;
        }

        if (rx_block == NULL || rx_size == 0) {
            if(rx_block != NULL) {
                k_mem_slab_free(&rx_mem_slab, rx_block);
            }
            continue;
        }

        /***************** TX PROCESS *****************/
        // Prepare TX block
        void *tx_block;
        ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
        if (ret < 0) {
            printk("Failed to allocate TX block: %d\n", ret);
            k_mem_slab_free(&rx_mem_slab, rx_block); // Free RX block if TX allocation fails
            continue;
        }

        // For loopback demonstration, simply copy RX data to TX block
        size_t block_size = rx_size < BLOCK_SIZE ? rx_size : BLOCK_SIZE;
        memcpy(tx_block, rx_block, block_size);

        // Free RX block back to slab
        k_mem_slab_free(&rx_mem_slab, rx_block);

        // Queue TX block for transmission
        ret = i2s_write(i2s_tx_device, tx_block, BLOCK_SIZE);
        if (ret < 0) {
            printk("Failed to queue TX block: %d\n", ret);
            k_mem_slab_free(&tx_mem_slab, tx_block);
            continue;
        }
    }
}

static void audio_stream(void){
    int ret;
    static uint16_t frame_id = 0;
    while(1){
        void *rx_block;
        size_t rx_size;

        // Read audio data from RX into rx memory block
        ret = i2s_read(i2s_rx_device, &rx_block, &rx_size);
        if (ret < 0) {
            printk("Failed to read RX block: %d\n", ret);
            continue;
        }

        if (rx_block == NULL || rx_size == 0) {
            if(rx_block != NULL) {
                k_mem_slab_free(&rx_mem_slab, rx_block);
            }
            continue;
        }

        // Send received audio data over UDP
        audio_pkt_t pkt;
        pkt.magic = AUDIO_MAGIC;
        pkt.version = AUDIO_VERSION;
        pkt.channels = 1; // Mono for now
        pkt.sample_rate = SAMPLE_FREQUENCY;
        pkt.frame_id = frame_id++;

        int16_t *pcm_data = (int16_t *)rx_block;
        for (int i = 0; i < AUDIO_SAMPLES_PER_FRAME; i++)
        {
            pkt.pcm[i] = pcm_data[i * NUMBER_OF_CHANNELS]; // Take only one channel
        }
        
        k_mem_slab_free(&rx_mem_slab, rx_block);

        ret = udp_send(&pkt, sizeof(pkt));;
        if (ret < 0) {
            printk("Failed to send UDP packet: %d\n", ret);
            continue;
        }
    }
}

static void audio_thread(void *a, void *b, void *c){
    audio_loopback();
}

void start_audio_thread(const struct device *i2s_tx,
                        const struct device *i2s_rx,
                        const struct i2s_config *config_tx,
                        const struct i2s_config *config_rx)
{
    i2s_tx_device = i2s_tx;
    i2s_rx_device = i2s_rx;
    static struct i2s_config tx_config;
    static struct i2s_config rx_config;

    tx_config = *config_tx;
    rx_config = *config_rx;

    tx_config.mem_slab = &tx_mem_slab;
    rx_config.mem_slab = &rx_mem_slab;

    int ret;

    ret = audio_init(i2s_tx, i2s_rx, &tx_config, &rx_config);
    if (ret < 0) {
        printk("Audio initialization failed: %d\n", ret);
        return;
    }

    ret = audio_start(i2s_tx, i2s_rx);
    if (ret < 0) {
        printk("Audio start failed: %d\n", ret);
        return;
    }

    k_thread_create(&audio_thread_data, audio_thread_stack,
                    K_THREAD_STACK_SIZEOF(audio_thread_stack),
                    audio_thread,
                    NULL, NULL, NULL,
                    AUDIO_THREAD_PRIORITY, 0, K_NO_WAIT);
}