#pragma once
#include <stdint.h>

#define AUDIO_MAGIC   0xA0D1
#define AUDIO_VERSION 1

/* 10 ms @ 48 kHz mono => 480 samples */
#define AUDIO_SAMPLES_PER_FRAME 480

typedef struct {
    uint16_t magic;        // = AUDIO_MAGIC
    uint8_t  version;      // = AUDIO_VERSION
    uint8_t  channels;     // 1 for now
    uint16_t sample_rate;  // 48000
    uint16_t frame_id;     // increments each packet
    int16_t  pcm[AUDIO_SAMPLES_PER_FRAME]; // mono PCM16
}__attribute__((packed)) audio_pkt_t;

typedef struct {
    uint8_t version:2;
    uint8_t padding:1;
    uint8_t extension:1;
    uint8_t csrc_count:4;
    uint8_t marker:1;
    uint8_t payload_type:7;
    uint16_t sequence_number;
    uint32_t timestamp;
    uint32_t ssrc;
}__attribute__((packed))rtp_t;