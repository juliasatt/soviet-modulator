#ifndef BPSK_MODULATOR_H
#define BPSK_MODULATOR_H

#include <Arduino.h>
#include <hardware/pio.h>

#define FRAME_SIZE 1024
#define CCSDS_ASM_SIZE 4
#define CCSDS_ASM 0x1ACFFC1D
#define MSG_BUFFER_SIZE 512

// PIO program for BPSK modulator
// Simple program: pull byte, shift out 8 bits to pin
static const uint16_t bpsk_modulator_program[] = {
    0x80a0,  // 0: pull   noblock
    0xa022,  // 1: mov    x, ~null     (load bit counter: 31 -> will loop 8 times for 8-bit shift)
    0xb040,  // 2: out    pins, 1      (shift out 1 bit to pin)
    0x0082,  // 3: jmp    x-- 2        (loop until done)
};

static const pio_program_t bpsk_modulator_program_default = {
    .instructions = bpsk_modulator_program,
    .length = 4,
    .origin = -1,
};

class BPSKModulator {
private:
    PIO pio;
    uint sm;
    uint32_t symbolrate_hz;
    uint pio_offset;
    uint16_t msg_len;
    bool msg_pending;

public:
    uint8_t frame[FRAME_SIZE];
    uint8_t msg_buffer[MSG_BUFFER_SIZE];
    BPSKModulator(uint pin, uint32_t initial_symbolrate = 1000) 
        : pio(pio0), sm(0), symbolrate_hz(initial_symbolrate), msg_len(0), msg_pending(false) {
        init_frame();
        init_pio(pin);
    }

    void init_frame() {
        // CCSDS ASM (4 bytes)
        frame[0] = (CCSDS_ASM >> 24) & 0xFF;
        frame[1] = (CCSDS_ASM >> 16) & 0xFF;
        frame[2] = (CCSDS_ASM >> 8) & 0xFF;
        frame[3] = CCSDS_ASM & 0xFF;

        // Rest: repeated 0x55 (01010101 pattern)
        for (uint16_t i = CCSDS_ASM_SIZE; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_frame_with_message(const uint8_t* msg, uint16_t len) {
        // CCSDS ASM (4 bytes)
        frame[0] = (CCSDS_ASM >> 24) & 0xFF;
        frame[1] = (CCSDS_ASM >> 16) & 0xFF;
        frame[2] = (CCSDS_ASM >> 8) & 0xFF;
        frame[3] = CCSDS_ASM & 0xFF;

        // Copy message
        uint16_t pos = CCSDS_ASM_SIZE;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++) {
            frame[pos] = msg[i];
        }

        // Pad rest with 0x55 (10101010)
        for (uint16_t i = pos; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_frame_message_only(const uint8_t* msg, uint16_t len) {
        // Message at beginning
        uint16_t pos = 0;
        for (uint16_t i = 0; i < len && pos < FRAME_SIZE; i++, pos++) {
            frame[pos] = msg[i];
        }

        // Pad rest with 0x55 (10101010 pattern)
        for (uint16_t i = pos; i < FRAME_SIZE; i++) {
            frame[i] = 0x55;
        }
    }

    void init_pio(uint pin) {
        // Load PIO program
        pio_offset = pio_add_program(pio, &bpsk_modulator_program_default);

        // Initialize state machine config
        pio_sm_config c = pio_get_default_sm_config();
        
        // Set output pin (shift out to this pin)
        sm_config_set_out_pins(&c, pin, 1);
        
        // Right shift, autopull after 8 bits
        sm_config_set_out_shift(&c, true, true, 8);

        // Initialize and start SM
        pio_sm_init(pio, sm, pio_offset, &c);
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        
        set_symbolrate(symbolrate_hz);
        pio_sm_set_enabled(pio, sm, true);

        // Fill TX FIFO with initial data
        refill_fifo();
    }

    void refill_fifo() {
        // Non-blocking: push bytes only if FIFO has space
        // Don't wait for all bytes to be written
        for (uint16_t i = 0; i < FRAME_SIZE; i++) {
            if (!pio_sm_is_tx_fifo_full(pio, sm)) {
                pio_sm_put(pio, sm, frame[i]);
            } else {
                break;  // FIFO full, try again later
            }
        }
    }

    void queue_message(const uint8_t* msg, uint16_t len) {
        if (len > MSG_BUFFER_SIZE) len = MSG_BUFFER_SIZE;
        memcpy(msg_buffer, msg, len);
        msg_len = len;
        msg_pending = true;
    }

    bool has_pending_message() const {
        return msg_pending;
    }

    void inject_message(bool filler_enabled) {
        if (!msg_pending || msg_len == 0) return;

        if (filler_enabled) {
            init_frame_with_message(msg_buffer, msg_len);
        } else {
            init_frame_message_only(msg_buffer, msg_len);
        }

        msg_pending = false;
        refill_fifo();
    }

    void set_symbolrate(uint32_t hz) {
        if (hz < 1 || hz > 100000) return;

        symbolrate_hz = hz;

        // clkdiv = sys_clock / (symbolrate * 2)
        // sys_clock = 150 MHz for RP2350
        float clkdiv = 150000000.0f / (hz * 2.0f);

        pio_sm_set_clkdiv(pio, sm, clkdiv);
    }

    uint32_t get_symbolrate() const {
        return symbolrate_hz;
    }

    void start() {
        pio_sm_set_enabled(pio, sm, true);
        refill_fifo();
    }

    void stop() {
        pio_sm_set_enabled(pio, sm, false);
    }
};

#endif // BPSK_MODULATOR_H


