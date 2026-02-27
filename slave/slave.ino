/**
 * slave.ino — Remote Keyless Entry System: SLAVE (Door Unit)
 * ═══════════════════════════════════════════════════════════════════
 * Hardware : Arduino Nano (ATmega328P) + NRF24L01 (SPI) + DS3231 (I2C)
 *            + Servo Motor (PWM) + USB / wall-adapter power
 *
 * ── Key differences vs. the ESP32 version ────────────────────────
 * • EEPROM        instead of Preferences (NVS) for counter storage
 * • Manual HMAC-SHA256 via Crypto library (Rhys Weatherley / SHA256.h)
 *   instead of mbedTLS (ESP32 only)
 * • Standard Servo.h instead of ESP32Servo
 * • Wire.begin() with no args  — Nano I2C is fixed: SDA=A4, SCL=A5
 * • No esp_sleep (Slave is mains-powered; no sleep needed)
 * • F() macro used on string literals to keep them in Flash (PROGMEM)
 *   and preserve the Nano's 2 KB SRAM
 * • No Serial.printf — DBG_PRINTF in shared_config.h uses snprintf
 *   on non-ESP32 targets
 *
 * ── RAM budget (ATmega328P, 2048 bytes) ──────────────────────────
 * Globals: RF24(~32B) + RTC(~4B) + Servo(~8B) + SHA256 ctx(~104B)
 *        + g_last_counter(4B) + replay_cache(164B) ≈ 316 B
 * Stack headroom: >1700 B — adequate for all local buffers below.
 *
 * Behaviour:
 *   • Boots, loads last-valid counter from EEPROM.
 *   • Continuously polls NRF24L01 for incoming RKE packets.
 *   • On packet: runs a 5-layer validation pipeline; on VALID rotates
 *     servo to unlock, holds 5 s, relocks, saves counter to EEPROM.
 *   • Anti-replay: consumed counters are blacklisted immediately.
 *
 * Libraries required (install via Arduino Library Manager):
 *   • RF24        — search "RF24" by nRF24
 *   • RTClib      — search "RTClib" by Adafruit
 *   • Crypto      — search "Crypto" by Rhys Weatherley (arduinolibs)
 *   Servo.h and EEPROM.h are bundled with the Arduino AVR core.
 *
 * Board: Arduino Nano — select "Arduino Nano" in Tools → Board.
 *   For older bootloader Nanos: Tools → Processor → "ATmega328P (Old Bootloader)"
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
#include <EEPROM.h>       // AVR non-volatile storage
#include <Servo.h>        // Standard servo library (bundled with Arduino core)

// Crypto library by Rhys Weatherley for SHA256
// Install via Library Manager: search "Crypto" by Rhys Weatherley
#include <SHA256.h>

// ─── Pin Definitions (Arduino Nano / ATmega328P) ──────────────────
// SPI bus for NRF24L01:
//   MOSI = D11  (hardware SPI)
//   MISO = D12  (hardware SPI)
//   SCK  = D13  (hardware SPI — also drives the Nano's built-in LED)
//   CSN  = D10  (hardware SS — also used as chip-select for NRF24)
//   CE   = D9   (chip-enable — any digital output is fine)
//
// ⚠ Avoid using D11 / D12 / D13 for anything other than SPI.
// ⚠ D13 = built-in LED — will flicker during SPI transfers (normal).
// ⚠ Do NOT use D0 / D1 (UART RX/TX) as GPIO while Serial is active.
#define PIN_NRF_CE    9
#define PIN_NRF_CSN  10

// I2C for DS3231 — fixed hardware pins on Nano, no need to pass to Wire.begin()
//   SDA = A4,  SCL = A5

// Servo PWM output.
// D6 is a 490 Hz PWM-capable pin on Nano, safe from hardware SPI/I2C.
// ⚠ Power the servo from an external 5 V rail — NOT the Nano's 5 V pin.
//   USB-sourced 5 V can supply only ~400 mA total; a servo stall can
//   exceed this and brown-out the Nano.  Tie servo GND to Nano GND.
#define PIN_SERVO     6

// ─── EEPROM Layout ────────────────────────────────────────────────
// ATmega328P has 1024 bytes of EEPROM (rated ~100,000 write cycles).
// The slave only writes on a valid unlock event, so wear is minimal.
#define EEPROM_ADDR_MAGIC   0   // 4 bytes — detects first-boot blank EEPROM
#define EEPROM_ADDR_CTR     4   // 4 bytes — last validated counter
#define EEPROM_MAGIC_VAL    0xAB3C1D2EUL

// ─── Global Objects ────────────────────────────────────────────────
RF24        radio(PIN_NRF_CE, PIN_NRF_CSN);
RTC_DS3231  rtc;
Servo       doorServo;

// SHA256 context kept in global BSS (not stack) to save stack space
// during HMAC computation.  NOT reentrant, but single-threaded is fine.
static SHA256 g_sha;

// ─── State Variables ──────────────────────────────────────────────
static uint32_t g_last_counter = 0;

// Anti-replay cache: tracks consumed counter values within the acceptance
// window so the same code cannot unlock twice.
// Size = 2*COUNTER_WINDOW+1 = 41 entries × 4 bytes = 164 bytes in SRAM.
#define REPLAY_CACHE_SIZE  (2 * COUNTER_WINDOW + 1)
static uint32_t g_replay_cache[REPLAY_CACHE_SIZE];
static uint8_t  g_replay_count = 0;

// ─── EEPROM Helpers ───────────────────────────────────────────────
/**
 * eeprom_load_counter()
 *
 * Reads the last validated counter from EEPROM.
 * Detects a blank/first-boot EEPROM via a magic word and initialises
 * counter to 0 if needed.
 * EEPROM.get() uses EEPROM.read() internally and is multi-byte safe.
 */
static void eeprom_load_counter(void) {
    uint32_t magic = 0;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);

    if (magic != EEPROM_MAGIC_VAL) {
        // First boot — write magic and reset counter
        DBG_PRINTLN(F("[EEPROM] First boot detected — initialising."));
        EEPROM.put(EEPROM_ADDR_MAGIC, (uint32_t)EEPROM_MAGIC_VAL);
        EEPROM.put(EEPROM_ADDR_CTR,   (uint32_t)0);
        g_last_counter = 0;
    } else {
        EEPROM.get(EEPROM_ADDR_CTR, g_last_counter);
        DBG_PRINTF("[EEPROM] Loaded last counter = %lu\n",
                   (unsigned long)g_last_counter);
    }
}

/**
 * eeprom_save_counter()
 *
 * Persists g_last_counter to EEPROM.
 * EEPROM.put() compares each byte before writing (equivalent to
 * EEPROM.update()), reducing unnecessary write cycles.
 */
static void eeprom_save_counter(void) {
    EEPROM.put(EEPROM_ADDR_CTR, g_last_counter);
    DBG_PRINTF("[EEPROM] Saved last counter = %lu\n",
               (unsigned long)g_last_counter);
}

// ─── Anti-Replay Cache ────────────────────────────────────────────
static bool replay_cache_contains(uint32_t counter) {
    for (uint8_t i = 0; i < g_replay_count; i++) {
        if (g_replay_cache[i] == counter) return true;
    }
    return false;
}

static void replay_cache_add(uint32_t counter) {
    if (g_replay_count < REPLAY_CACHE_SIZE) {
        g_replay_cache[g_replay_count++] = counter;
    } else {
        // Circular eviction — drop oldest entry
        memmove(g_replay_cache, g_replay_cache + 1,
                (REPLAY_CACHE_SIZE - 1) * sizeof(uint32_t));
        g_replay_cache[REPLAY_CACHE_SIZE - 1] = counter;
    }
}

static void replay_cache_reset(void) {
    g_replay_count = 0;
    memset(g_replay_cache, 0, sizeof(g_replay_cache));
}

// ─── HMAC-SHA256 (Manual, using Crypto library) ───────────────────
/**
 * compute_hmac()
 *
 * Computes HMAC-SHA256(SECRET_SEED, device_id || counter || timestamp)
 * using the two-pass construction:
 *   inner = SHA256( (K XOR ipad) || message )
 *   hmac  = SHA256( (K XOR opad) || inner )
 *
 * The SECRET_SEED (32 bytes) is shorter than the SHA256 block size
 * (64 bytes), so it is zero-padded to 64 bytes before XOR — done
 * inline by clamping the loop.
 *
 * Only the first 16 bytes of the 32-byte digest are stored in the
 * packet (sufficient for 128-bit authentication on this link).
 *
 * ⚠ g_sha is a file-scope static to avoid putting 104 bytes of SHA256
 *   context on the stack inside this function.
 *
 * Stack usage of this function: message[12] + k_xor[64] + inner_hash[32]
 *                               = 108 bytes  (well within Nano headroom)
 *
 * @param device_id   4-byte master identifier
 * @param counter     rolling code counter value
 * @param timestamp   Unix time from RTC
 * @param out_hmac    output buffer, must be at least 16 bytes
 */
static void compute_hmac(uint32_t device_id, uint32_t counter,
                          uint32_t timestamp, uint8_t *out_hmac) {

    // Build the 12-byte message: device_id || counter || timestamp
    uint8_t message[12];
    memcpy(message + 0, &device_id, 4);
    memcpy(message + 4, &counter,   4);
    memcpy(message + 8, &timestamp, 4);

    // 64-byte key pads (SHA256 block size).
    // We reuse a single k_xor buffer for both ipad and opad passes.
    uint8_t k_xor[64];
    uint8_t inner_hash[32];

    // ── Pass 1: inner hash = SHA256( K⊕ipad || message ) ─────────
    // K⊕ipad: for i < 32 XOR SECRET_SEED[i] with 0x36; for i >= 32 use 0x36
    for (uint8_t i = 0; i < 64; i++) {
        k_xor[i] = (i < 32) ? (SECRET_SEED[i] ^ 0x36) : 0x36;
    }
    g_sha.reset();
    g_sha.update(k_xor, 64);
    g_sha.update(message, 12);
    g_sha.finalize(inner_hash, 32);

    // ── Pass 2: outer hash = SHA256( K⊕opad || inner_hash ) ──────
    // Reuse k_xor buffer for opad (0x5C instead of 0x36)
    for (uint8_t i = 0; i < 64; i++) {
        k_xor[i] = (i < 32) ? (SECRET_SEED[i] ^ 0x5C) : 0x5C;
    }
    g_sha.reset();
    g_sha.update(k_xor, 64);
    g_sha.update(inner_hash, 32);
    // Write directly to out_hmac — SHA256::finalize() copies min(len, 32) bytes
    g_sha.finalize(out_hmac, 16);
}

// ─── Packet Validation ────────────────────────────────────────────
// ValidationResult enum is defined in shared_config.h so the Arduino
// IDE can see it before it injects auto-generated function prototypes.

static const __FlashStringHelper* val_result_str(ValidationResult r) {
    switch (r) {
        case VALID:          return F("VALID");
        case ERR_CHECKSUM:   return F("CHECKSUM_FAIL");
        case ERR_DEVICE_ID:  return F("DEVICE_ID_MISMATCH");
        case ERR_COUNTER:    return F("COUNTER_OUT_OF_WINDOW");
        case ERR_REPLAY:     return F("REPLAY_DETECTED");
        case ERR_HMAC:       return F("HMAC_MISMATCH");
        case ERR_TIMESTAMP:  return F("TIMESTAMP_STALE");
        default:             return F("UNKNOWN");
    }
}

/**
 * validate_packet()
 *
 * 5-layer validation pipeline — ordered cheapest to most expensive.
 * Layers 1–3 are O(1) integer operations. Layer 4 is an O(n) scan
 * (n ≤ 41). Layer 5 (HMAC) is the only cryptographic operation and
 * only runs after all cheap checks pass.
 */
static ValidationResult validate_packet(const RKEPacket *pkt,
                                        uint32_t now_unix) {

    // ── Layer 1: XOR Checksum ────────────────────────────────────
    uint32_t expected_crc = 0;
    const uint8_t *raw = (const uint8_t *)pkt;
    for (uint8_t i = 0; i < 28; i++) {
        expected_crc ^= ((uint32_t)raw[i]) << ((i % 4) * 8);
    }
    if (expected_crc != pkt->checksum) {
        DBG_PRINT(F("[VAL] Checksum fail: got 0x"));
        DBG_PRINT(pkt->checksum, HEX);
        DBG_PRINT(F(" expected 0x"));
        DBG_PRINTLN(expected_crc, HEX);
        return ERR_CHECKSUM;
    }

    // ── Layer 2: Device ID ────────────────────────────────────────
    if (pkt->device_id != DEVICE_ID) {
        DBG_PRINT(F("[VAL] Device ID mismatch: got 0x"));
        DBG_PRINT(pkt->device_id, HEX);
        DBG_PRINT(F(" want 0x"));
        DBG_PRINTLN((uint32_t)DEVICE_ID, HEX);
        return ERR_DEVICE_ID;
    }

    // ── Layer 3: Counter Window ───────────────────────────────────
    // Use signed 32-bit arithmetic to handle window near counter = 0.
    int32_t delta = (int32_t)(pkt->counter) - (int32_t)(g_last_counter);
    if (delta < -(int32_t)COUNTER_WINDOW || delta > (int32_t)COUNTER_WINDOW) {
        DBG_PRINT(F("[VAL] Counter out of window: recv="));
        DBG_PRINT(pkt->counter);
        DBG_PRINT(F(" last="));
        DBG_PRINT(g_last_counter);
        DBG_PRINT(F(" delta="));
        DBG_PRINTLN(delta);
        return ERR_COUNTER;
    }

    // ── Layer 4: Anti-Replay Cache ────────────────────────────────
    if (replay_cache_contains(pkt->counter)) {
        DBG_PRINT(F("[VAL] Replay detected! Counter "));
        DBG_PRINT(pkt->counter);
        DBG_PRINTLN(F(" already used."));
        return ERR_REPLAY;
    }

    // ── Layer 5: HMAC Verification ────────────────────────────────
    uint8_t expected_hmac[16];
    compute_hmac(pkt->device_id, pkt->counter, pkt->timestamp, expected_hmac);

    if (memcmp(pkt->hmac, expected_hmac, 16) != 0) {
        DBG_PRINTLN(F("[VAL] HMAC mismatch — packet rejected."));
        return ERR_HMAC;
    }

    // ── Layer 6: Timestamp Freshness ─────────────────────────────
    // Reject packets more than TIMESTAMP_TOLERANCE seconds old or future.
    int32_t ts_delta = (int32_t)now_unix - (int32_t)pkt->timestamp;
    if (ts_delta < -(int32_t)TIMESTAMP_TOLERANCE ||
        ts_delta >  (int32_t)TIMESTAMP_TOLERANCE) {
        DBG_PRINT(F("[VAL] Timestamp stale: pkt_ts="));
        DBG_PRINT(pkt->timestamp);
        DBG_PRINT(F(" now="));
        DBG_PRINT(now_unix);
        DBG_PRINT(F(" delta="));
        DBG_PRINTLN(ts_delta);
        return ERR_TIMESTAMP;
    }

    return VALID;
}

// ─── Servo Control ───────────────────────────────────────────────
static void servo_unlock(void) {
    DBG_PRINTLN(F("[SERVO] Unlocking door..."));
    doorServo.write(SERVO_UNLOCKED_DEG);
    delay(UNLOCK_DURATION_MS);
    doorServo.write(SERVO_LOCKED_DEG);
    DBG_PRINTLN(F("[SERVO] Door re-locked."));
}

// ─── NRF24L01 Initialisation ─────────────────────────────────────
static bool init_radio(void) {
    if (!radio.begin()) {
        DBG_PRINTLN(F("[Radio] NRF24L01 init FAILED — check wiring!"));
        return false;
    }
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF_DATA_RATE);
    radio.setPALevel(RF_PA_LEVEL);
    radio.setPayloadSize(RF_PAYLOAD_SIZE);
    radio.setAutoAck(false);
    radio.openReadingPipe(1, PIPE_ADDRESS);
    radio.startListening();  // RX mode
    DBG_PRINTLN(F("[Radio] NRF24L01 ready (RX mode)"));
    return true;
}

// ─── DS3231 RTC Initialisation ────────────────────────────────────
static bool init_rtc(void) {
    // Wire.begin() with no arguments uses Nano's fixed I2C pins (A4=SDA, A5=SCL)
    Wire.begin();
    if (!rtc.begin()) {
        DBG_PRINTLN(F("[RTC] DS3231 not found — check I2C wiring!"));
        return false;
    }
    if (rtc.lostPower()) {
        DBG_PRINTLN(F("[RTC] RTC lost power — seeding with compile time"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    return true;
}

// ─── Arduino Setup ────────────────────────────────────────────────
void setup(void) {
#ifdef DEBUG
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n+---------------------------------+"));
    Serial.println(F("|  RKE Slave (Nano) — Door Boot  |"));
    Serial.println(F("+---------------------------------+"));
#endif

    // Servo — attach and set locked position immediately on boot
    // Pulse range 500–2400 µs covers most hobby servos
    doorServo.attach(PIN_SERVO, 500, 2400);
    doorServo.write(SERVO_LOCKED_DEG);
    DBG_PRINT(F("[Servo] Attached on D"));
    DBG_PRINT(PIN_SERVO);
    DBG_PRINTLN(F(" — LOCKED"));

    if (!init_rtc()) {
        DBG_PRINTLN(F("[FATAL] RTC init failed. Halting."));
        while (true) delay(1000);
    }

    eeprom_load_counter();

    if (!init_radio()) {
        DBG_PRINTLN(F("[FATAL] Radio init failed. Halting."));
        while (true) delay(1000);
    }

    DBG_PRINTLN(F("[Setup] Slave ready. Listening...\n"));
}

// ─── Arduino Loop ─────────────────────────────────────────────────
void loop(void) {
    if (!radio.available()) {
        return;  // Nothing in FIFO — keep polling
    }

    // ── 1. Read packet from radio FIFO ───────────────────────────
    RKEPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    DBG_PRINTLN(F("----------------------------------"));
    DBG_PRINT(F("[RX] DevID=0x"));
    DBG_PRINT(pkt.device_id, HEX);
    DBG_PRINT(F("  Ctr="));
    DBG_PRINT(pkt.counter);
    DBG_PRINT(F("  TS="));
    DBG_PRINTLN(pkt.timestamp);

    // ── 2. Get current time for timestamp freshness check ────────
    DateTime now      = rtc.now();
    uint32_t now_unix = now.unixtime();

    // ── 3. Validate ───────────────────────────────────────────────
    ValidationResult result = validate_packet(&pkt, now_unix);

    if (result != VALID) {
        DBG_PRINT(F("[RX] REJECTED — "));
        DBG_PRINTLN(val_result_str(result));
        return;
    }

    // ── 4. Valid packet — mark counter as consumed ────────────────
    DBG_PRINTLN(F("[RX] Packet VALID — Unlocking!"));
    replay_cache_add(pkt.counter);

    // Advance Slave's base counter if Master is ahead
    if (pkt.counter >= g_last_counter) {
        replay_cache_reset();
        replay_cache_add(pkt.counter);   // Re-add after reset
        g_last_counter = pkt.counter;
        eeprom_save_counter();
    }

    // ── 5. Unlock sequence ────────────────────────────────────────
    servo_unlock();

    DBG_PRINTLN(F("[RX] Cycle complete. Listening again.\n"));
}
