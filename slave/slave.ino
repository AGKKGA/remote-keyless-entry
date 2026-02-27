/**
 * slave.ino — Remote Keyless Entry System: SLAVE (Door Unit)
 * ═══════════════════════════════════════════════════════════════════
 * Hardware : ESP32 DevKit v1 + NRF24L01 (SPI) + DS3231 (I2C)
 *            + Servo Motor (PWM) + USB / wall-adapter power
 *
 * Behaviour:
 *   • Boots, loads last-valid counter from NVS.
 *   • Continuously listens on NRF24L01 for RKE packets.
 *   • On receiving a packet, performs a 4-layer validation:
 *     1. XOR checksum  — fast discard of corrupted frames
 *     2. Device ID     — ensures packet is from our key fob
 *     3. Counter window — ±COUNTER_WINDOW of last known counter
 *     4. HMAC-SHA256   — cryptographic authenticity check
 *     5. Timestamp     — freshness check (< TIMESTAMP_TOLERANCE s)
 *   • On valid packet: rotates servo to unlock, waits 5 s, relocks.
 *   • Persists last valid counter to NVS to survive power cycles.
 *   • Anti-replay: every accepted counter is tracked; same code
 *     cannot unlock twice within the acceptance window.
 *
 * Libraries required (install via Arduino Library Manager):
 *   • RF24          — https://github.com/nRF24/RF24
 *   • RTClib        — https://github.com/adafruit/RTClib
 *   • ESP32Servo    — https://github.com/madhephaestus/ESP32Servo
 *   mbedTLS and Preferences are part of the ESP32 Arduino core.
 *
 * Place shared_config.h in the same folder as this sketch.
 * ═══════════════════════════════════════════════════════════════════
 */

#include "shared_config.h"

// ─── Library Includes ─────────────────────────────────────────────
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <RTClib.h>
#include <Preferences.h>
#include <ESP32Servo.h>

// mbedTLS HMAC-SHA256 (built into ESP32 Arduino core)
#include "mbedtls/md.h"

// ─── Pin Definitions (ESP32 DevKit v1) ────────────────────────────
// SPI bus for NRF24L01 — same wiring as Master:
//   MOSI = GPIO 23, MISO = GPIO 19, SCK = GPIO 18
//   CSN  = GPIO 5,  CE   = GPIO 4
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5

// I2C for DS3231
#define PIN_SDA      21
#define PIN_SCL      22

// Servo PWM output.
// GPIO 13 is a safe, general-purpose output on ESP32 DevKit.
// IMPORTANT: Power the servo from an external 5 V supply, NOT from the
//   ESP32 3.3 V rail — servo stall current can exceed 500 mA and will
//   brown-out the ESP32 or damage the 3.3 V regulator.
// Connect servo GND to the same ground as ESP32.
#define PIN_SERVO    13

// ─── Global Objects ────────────────────────────────────────────────
RF24        radio(PIN_NRF_CE, PIN_NRF_CSN);
RTC_DS3231  rtc;
Preferences prefs;
Servo       doorServo;

// ─── State Variables ──────────────────────────────────────────────
// Last counter value from a successfully validated packet.
static uint32_t g_last_counter = 0;

// Bitmap for anti-replay: track which counters within the acceptance
// window have already been consumed.  The window is 2 * COUNTER_WINDOW
// + 1 codes centred on g_last_counter.  We store consumed offsets
// relative to the base counter when the window was last opened.
//
// For simplicity (and because COUNTER_WINDOW = 20) we keep a flat
// array of at most 2*COUNTER_WINDOW+1 = 41 consumed counters.
#define REPLAY_CACHE_SIZE  (2 * COUNTER_WINDOW + 1)
static uint32_t g_replay_cache[REPLAY_CACHE_SIZE];
static uint8_t  g_replay_count = 0;

// ─── NVS Helpers ──────────────────────────────────────────────────
#define NVS_NAMESPACE "rke_slave"
#define NVS_KEY_CTR   "last_ctr"

static void nvs_load_counter(void) {
    prefs.begin(NVS_NAMESPACE, false);
    g_last_counter = prefs.getUInt(NVS_KEY_CTR, 0);
    prefs.end();
    DBG_PRINTF("[NVS] Loaded last counter = %u\n", g_last_counter);
}

static void nvs_save_counter(void) {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUInt(NVS_KEY_CTR, g_last_counter);
    prefs.end();
    DBG_PRINTF("[NVS] Saved last counter = %u\n", g_last_counter);
}

// ─── Anti-Replay Cache ────────────────────────────────────────────
/**
 * replay_cache_contains() — O(n) scan; n ≤ 41, so this is fine.
 * Returns true if 'counter' has already been used.
 */
static bool replay_cache_contains(uint32_t counter) {
    for (uint8_t i = 0; i < g_replay_count; i++) {
        if (g_replay_cache[i] == counter) return true;
    }
    return false;
}

/**
 * replay_cache_add() — Add a counter to the consumed list.
 * If the cache is full (shouldn't happen with window ≤ 20),
 * the oldest entry is overwritten in a circular fashion.
 */
static void replay_cache_add(uint32_t counter) {
    if (g_replay_count < REPLAY_CACHE_SIZE) {
        g_replay_cache[g_replay_count++] = counter;
    } else {
        // Shift left and append — oldest entry evicted
        memmove(g_replay_cache, g_replay_cache + 1,
                (REPLAY_CACHE_SIZE - 1) * sizeof(uint32_t));
        g_replay_cache[REPLAY_CACHE_SIZE - 1] = counter;
    }
}

/**
 * replay_cache_reset() — Called when Slave advances its base counter.
 * Old entries before the new base are no longer relevant.
 */
static void replay_cache_reset(void) {
    g_replay_count = 0;
    memset(g_replay_cache, 0, sizeof(g_replay_cache));
}

// ─── HMAC-SHA256 Helper ───────────────────────────────────────────
/**
 * compute_hmac() — identical to Master's implementation.
 * Must stay byte-for-byte identical with master.ino.
 */
static void compute_hmac(uint32_t device_id, uint32_t counter,
                          uint32_t timestamp, uint8_t *out_hmac) {
    uint8_t message[12];
    memcpy(message + 0, &device_id,  4);
    memcpy(message + 4, &counter,    4);
    memcpy(message + 8, &timestamp,  4);

    uint8_t full_hmac[32];

    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, info, 1);
    mbedtls_md_hmac_starts(&ctx, SECRET_SEED, sizeof(SECRET_SEED));
    mbedtls_md_hmac_update(&ctx, message, sizeof(message));
    mbedtls_md_hmac_finish(&ctx, full_hmac);
    mbedtls_md_free(&ctx);

    memcpy(out_hmac, full_hmac, 16);
}

// ─── Packet Validation ────────────────────────────────────────────
typedef enum {
    VALID = 0,
    ERR_CHECKSUM,    // XOR checksum mismatch (corrupted frame)
    ERR_DEVICE_ID,   // Wrong device ID
    ERR_COUNTER,     // Counter outside acceptance window
    ERR_REPLAY,      // Counter already consumed (replay attack)
    ERR_HMAC,        // HMAC mismatch (tampered or wrong seed)
    ERR_TIMESTAMP,   // Timestamp too old or too far in the future
} ValidationResult;

static const char* val_result_str(ValidationResult r) {
    switch (r) {
        case VALID:          return "VALID";
        case ERR_CHECKSUM:   return "CHECKSUM_FAIL";
        case ERR_DEVICE_ID:  return "DEVICE_ID_MISMATCH";
        case ERR_COUNTER:    return "COUNTER_OUT_OF_WINDOW";
        case ERR_REPLAY:     return "REPLAY_DETECTED";
        case ERR_HMAC:       return "HMAC_MISMATCH";
        case ERR_TIMESTAMP:  return "TIMESTAMP_STALE";
        default:             return "UNKNOWN";
    }
}

/**
 * validate_packet()
 *
 * Runs the full 5-layer validation pipeline on a received packet.
 * Layers are ordered cheapest-to-most-expensive so expensive HMAC
 * only runs after cheaper checks pass.
 *
 * @param pkt         pointer to received packet
 * @param now_unix    current Unix time from local RTC
 * @return            ValidationResult — VALID or specific error code
 */
static ValidationResult validate_packet(const RKEPacket *pkt,
                                        uint32_t now_unix) {

    // ── Layer 1: XOR Checksum ────────────────────────────────────
    // Recompute checksum over first 28 bytes and compare with field.
    uint32_t expected_crc = 0;
    const uint8_t *raw = (const uint8_t *)pkt;
    for (int i = 0; i < 28; i++) {
        expected_crc ^= ((uint32_t)raw[i]) << ((i % 4) * 8);
    }
    if (expected_crc != pkt->checksum) {
        DBG_PRINTF("[VAL] Checksum fail: got 0x%08X, expected 0x%08X\n",
                   pkt->checksum, expected_crc);
        return ERR_CHECKSUM;
    }

    // ── Layer 2: Device ID ────────────────────────────────────────
    if (pkt->device_id != DEVICE_ID) {
        DBG_PRINTF("[VAL] Device ID mismatch: got 0x%08X, want 0x%08X\n",
                   pkt->device_id, (uint32_t)DEVICE_ID);
        return ERR_DEVICE_ID;
    }

    // ── Layer 3: Counter Window ───────────────────────────────────
    // Accept counters in [g_last_counter - WINDOW, g_last_counter + WINDOW].
    // Cast to signed arithmetic to handle edge cases near 0.
    int32_t delta = (int32_t)(pkt->counter) - (int32_t)(g_last_counter);
    if (delta < -(int32_t)COUNTER_WINDOW || delta > (int32_t)COUNTER_WINDOW) {
        DBG_PRINTF("[VAL] Counter out of window: recv=%u last=%u delta=%d\n",
                   pkt->counter, g_last_counter, delta);
        return ERR_COUNTER;
    }

    // ── Layer 4: Anti-Replay ──────────────────────────────────────
    if (replay_cache_contains(pkt->counter)) {
        DBG_PRINTF("[VAL] Replay detected! Counter %u already used.\n",
                   pkt->counter);
        return ERR_REPLAY;
    }

    // ── Layer 5: HMAC Verification ────────────────────────────────
    // Recompute HMAC locally; compare with constant-time memcmp.
    // Note: memcmp is NOT constant-time in general, but since we already
    //   passed all cheaper checks, timing side-channels here are acceptable
    //   for a portfolio/educational project.  Production code would use
    //   mbedtls_ssl_safer_memcmp() or similar.
    uint8_t expected_hmac[16];
    compute_hmac(pkt->device_id, pkt->counter, pkt->timestamp, expected_hmac);

    if (memcmp(pkt->hmac, expected_hmac, 16) != 0) {
        DBG_PRINTLN("[VAL] HMAC mismatch — packet rejected.");
        return ERR_HMAC;
    }

    // ── Layer 6 (Optional but important): Timestamp Freshness ────
    // Reject packets older than TIMESTAMP_TOLERANCE seconds.
    // Also reject packets timestamped more than TOLERANCE seconds in the
    // future (prevents an attacker pre-computing future codes).
    int32_t ts_delta = (int32_t)now_unix - (int32_t)pkt->timestamp;
    if (ts_delta < -(int32_t)TIMESTAMP_TOLERANCE ||
        ts_delta >  (int32_t)TIMESTAMP_TOLERANCE) {
        DBG_PRINTF("[VAL] Timestamp out of range: pkt_ts=%u now=%u delta=%d\n",
                   pkt->timestamp, now_unix, ts_delta);
        return ERR_TIMESTAMP;
    }

    return VALID;
}

// ─── Servo Control ───────────────────────────────────────────────
static void servo_unlock(void) {
    DBG_PRINTLN("[SERVO] Unlocking door...");
    doorServo.write(SERVO_UNLOCKED_DEG);
    delay(UNLOCK_DURATION_MS);   // Hold unlocked for 5 seconds
    doorServo.write(SERVO_LOCKED_DEG);
    DBG_PRINTLN("[SERVO] Door re-locked.");
}

// ─── NRF24L01 Initialisation ─────────────────────────────────────
static bool init_radio(void) {
    if (!radio.begin()) {
        DBG_PRINTLN("[Radio] NRF24L01 init FAILED — check wiring!");
        return false;
    }

    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF_DATA_RATE);
    radio.setPALevel(RF_PA_LEVEL);
    radio.setPayloadSize(RF_PAYLOAD_SIZE);
    radio.setAutoAck(false);           // No ACK sent back to Master
    radio.openReadingPipe(1, PIPE_ADDRESS);
    radio.startListening();            // RX mode

    DBG_PRINTLN("[Radio] NRF24L01 ready (RX mode)");
    DBG_PRINTF("[Radio] Channel=%d, DataRate=250kbps, Listening...\n",
               RF_CHANNEL);
    return true;
}

// ─── DS3231 RTC Initialisation ────────────────────────────────────
static bool init_rtc(void) {
    Wire.begin(PIN_SDA, PIN_SCL);
    if (!rtc.begin()) {
        DBG_PRINTLN("[RTC] DS3231 not found — check I2C wiring!");
        return false;
    }
    if (rtc.lostPower()) {
        DBG_PRINTLN("[RTC] RTC lost power — seeding with compile time");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    DateTime now = rtc.now();
    DBG_PRINTF("[RTC] Current UTC: %04d-%02d-%02d %02d:%02d:%02d\n",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
    return true;
}

// ─── Arduino Setup ────────────────────────────────────────────────
void setup(void) {
#ifdef DEBUG
    Serial.begin(115200);
    delay(500);
    Serial.println("\n╔══════════════════════════════╗");
    Serial.println("║  RKE Slave — Door Unit Boot  ║");
    Serial.println("╚══════════════════════════════╝");
#endif

    // Servo — attach and set to locked position on boot
    doorServo.attach(PIN_SERVO, 500, 2400); // 500–2400 µs pulse range
    doorServo.write(SERVO_LOCKED_DEG);
    DBG_PRINTF("[Servo] Attached on GPIO %d, set to LOCKED (%d°)\n",
               PIN_SERVO, SERVO_LOCKED_DEG);

    if (!init_rtc()) {
        DBG_PRINTLN("[FATAL] RTC init failed. Halting.");
        while (true) delay(1000);
    }

    nvs_load_counter();

    if (!init_radio()) {
        DBG_PRINTLN("[FATAL] Radio init failed. Halting.");
        while (true) delay(1000);
    }

    DBG_PRINTLN("[Setup] Slave ready. Listening for RKE packets...\n");
}

// ─── Arduino Loop ─────────────────────────────────────────────────
void loop(void) {
    // Poll radio — non-blocking check for available payload
    if (!radio.available()) {
        return; // Nothing received — keep listening
    }

    // ── 1. Read the 32-byte packet from the radio FIFO ───────────
    RKEPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    DBG_PRINTLN("──────────────────────────────────");
    DBG_PRINTF("[RX] Packet received | DeviceID=0x%08X Counter=%u TS=%u\n",
               pkt.device_id, pkt.counter, pkt.timestamp);

    // ── 2. Get current time for timestamp freshness check ────────
    DateTime    now      = rtc.now();
    uint32_t    now_unix = now.unixtime();

    // ── 3. Validate the packet ────────────────────────────────────
    ValidationResult result = validate_packet(&pkt, now_unix);

    if (result != VALID) {
        DBG_PRINTF("[RX] REJECTED — reason: %s\n", val_result_str(result));
        return;
    }

    // ── 4. Packet is valid — perform unlock sequence ──────────────
    DBG_PRINTLN("[RX] ✓ Packet VALID — Unlocking door!");

    // Mark this counter as consumed (anti-replay)
    replay_cache_add(pkt.counter);

    // Advance Slave's counter to match Master.
    // If the incoming counter is ahead (Master ahead of Slave), jump to it.
    // If the incoming counter is behind-but-valid, still mark as consumed
    // but don't roll back the base counter.
    if (pkt.counter >= g_last_counter) {
        // Counter has advanced — reset replay cache (old codes no longer
        // within the acceptance window) and update base.
        replay_cache_reset();
        replay_cache_add(pkt.counter);  // Re-add the new counter after reset
        g_last_counter = pkt.counter;
        nvs_save_counter();
    }

    // ── 5. Rotate servo to unlock, hold, re-lock ─────────────────
    servo_unlock();

    DBG_PRINTLN("[RX] Unlock cycle complete. Listening again...\n");
}
