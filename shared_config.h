/**
 * shared_config.h
 * ─────────────────────────────────────────────────────────────────
 * Shared constants for the Remote Keyless Entry (RKE) system.
 * THIS FILE MUST BE IDENTICAL ON BOTH MASTER AND SLAVE.
 *
 * Flash this file together with master.ino or slave.ino.
 * The SECRET_SEED is never transmitted — it exists only in flash.
 * ─────────────────────────────────────────────────────────────────
 */

#ifndef SHARED_CONFIG_H
#define SHARED_CONFIG_H

#include <stdint.h>

// ─── Device Identity ──────────────────────────────────────────────
// Unique 4-byte ID of this key fob (Master).
// Must match on both sides. Change for each unique key fob.
#define DEVICE_ID  0xDEADBEEF

// ─── Rolling Code Secret ──────────────────────────────────────────
// 32-byte secret seed used as the HMAC key.
// NEVER transmitted over the air. Hardcoded at flash time only.
// Change this to a random value unique to your deployment.
static const uint8_t SECRET_SEED[32] = {
    0x3A, 0x7F, 0x12, 0xC4, 0x9E, 0x55, 0xB1, 0x08,
    0xFA, 0x23, 0x6D, 0x90, 0x47, 0xE3, 0x1B, 0xCD,
    0x82, 0x5C, 0xA0, 0x71, 0xD4, 0x38, 0xF6, 0x2E,
    0x0B, 0x9A, 0x64, 0x17, 0x5E, 0xCC, 0x89, 0x43
};

// ─── NRF24L01 Configuration ───────────────────────────────────────
// 5-byte pipe address shared between Master (TX) and Slave (RX).
static const uint8_t PIPE_ADDRESS[5] = { 0x52, 0x4B, 0x45, 0x4B, 0x59 }; // "RKEKY"

#define RF_CHANNEL          108   // 2.508 GHz — avoids WiFi channels
#define RF_DATA_RATE        RF24_250KBPS
#define RF_PA_LEVEL         RF24_PA_MAX
#define RF_PAYLOAD_SIZE     32

// ─── Rolling Code Window ──────────────────────────────────────────
// Slave accepts Master counter within this many steps of its own counter.
// Handles desync from missed packets while limiting replay window.
#define COUNTER_WINDOW      20

// ─── Timing ───────────────────────────────────────────────────────
#define TX_INTERVAL_MS        500    // Master transmit interval (ms)
#define TIMESTAMP_TOLERANCE   60     // Max RTC skew allowed (seconds)
#define UNLOCK_DURATION_MS    5000   // Servo stays unlocked for 5 seconds

// ─── Servo Positions ──────────────────────────────────────────────
#define SERVO_LOCKED_DEG    0     // Locked position in degrees
#define SERVO_UNLOCKED_DEG  90    // Unlocked position in degrees

// ─── Packet Structure (32 bytes total) ────────────────────────────
// Packed to avoid compiler padding between fields.
typedef struct __attribute__((packed)) {
    uint32_t device_id;       // 4 bytes — Master identifier
    uint32_t counter;         // 4 bytes — Rolling code counter
    uint8_t  hmac[16];        // 16 bytes — First 16 bytes of HMAC-SHA256
    uint32_t timestamp;       // 4 bytes — Unix time from RTC
    uint32_t checksum;        // 4 bytes — Simple XOR checksum of above 28 bytes
} RKEPacket;

// Verify packet size at compile time
static_assert(sizeof(RKEPacket) == RF_PAYLOAD_SIZE,
              "RKEPacket size must equal RF_PAYLOAD_SIZE (32 bytes)");

// ─── Debug Toggle ─────────────────────────────────────────────────
// Comment out to disable Serial debug output in production builds.
#define DEBUG

#ifdef DEBUG
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
#endif

#endif // SHARED_CONFIG_H
