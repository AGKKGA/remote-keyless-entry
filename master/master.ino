/**
 * master.ino — Remote Keyless Entry System: MASTER (Key Fob)
 * ═══════════════════════════════════════════════════════════════════
 * Hardware : ESP32 DevKit v1 + NRF24L01 (SPI) + DS3231 (I2C)
 *            + 18650 battery via TP4056 + 3.3 V regulator
 *
 * Behaviour:
 *   • Boots, loads rolling-code counter from NVS.
 *   • Every TX_INTERVAL_MS (500 ms) builds a 32-byte RKE packet,
 *     signs it with HMAC-SHA256(SECRET_SEED, counter), and
 *     broadcasts it via NRF24L01.
 *   • Increments counter and persists it to NVS after each TX.
 *   • Uses ESP32 light sleep between transmissions to save battery.
 *
 * Libraries required (install via Arduino Library Manager):
 *   • RF24          — https://github.com/nRF24/RF24
 *   • RTClib        — https://github.com/adafruit/RTClib
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

// mbedTLS HMAC-SHA256 (built into ESP32 Arduino core)
#include "mbedtls/md.h"

// ESP32 light-sleep API
#include "esp_sleep.h"

// ─── Pin Definitions (ESP32 DevKit v1) ────────────────────────────
// SPI bus for NRF24L01:
//   MOSI = GPIO 23  (VSPI MOSI)
//   MISO = GPIO 19  (VSPI MISO)
//   SCK  = GPIO 18  (VSPI CLK)
//   CSN  = GPIO 5   (chip-select, active low)
//   CE   = GPIO 4   (chip-enable)
//
// Avoid GPIO 6-11 (connected to internal flash).
// Avoid GPIO 34-39 (input-only, no internal pull-up).
// GPIO 0, 2, 12, 15 are strapping pins — safe at runtime but be
//   mindful during boot.
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5

// I2C for DS3231 RTC (ESP32 default Wire pins)
#define PIN_SDA      21
#define PIN_SCL      22

// ─── Global Objects ────────────────────────────────────────────────
RF24        radio(PIN_NRF_CE, PIN_NRF_CSN);  // CE, CSN
RTC_DS3231  rtc;
Preferences prefs;   // NVS namespace for persisting the counter

// ─── Rolling-Code Counter ─────────────────────────────────────────
// Loaded from NVS on boot; persisted after every transmission.
static uint32_t g_counter = 0;

// ─── NVS Helpers ──────────────────────────────────────────────────
// Namespace "rke" keeps our keys isolated from other sketches.
#define NVS_NAMESPACE "rke"
#define NVS_KEY_CTR   "counter"

/** Load counter from NVS. Defaults to 1 if namespace is empty. */
static void nvs_load_counter(void) {
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    g_counter = prefs.getUInt(NVS_KEY_CTR, 1);
    prefs.end();
    DBG_PRINTF("[NVS] Loaded counter = %u\n", g_counter);
}

/** Persist counter to NVS immediately after incrementing. */
static void nvs_save_counter(void) {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUInt(NVS_KEY_CTR, g_counter);
    prefs.end();
    DBG_PRINTF("[NVS] Saved counter = %u\n", g_counter);
}

// ─── HMAC-SHA256 Helper ───────────────────────────────────────────
/**
 * compute_hmac()
 *
 * Computes HMAC-SHA256 over the message:
 *   [ device_id (4 B) | counter (4 B) | timestamp (4 B) ]
 * using SECRET_SEED as the key.
 *
 * Only the first 16 bytes of the 32-byte digest are stored in the
 * packet to fit the 32-byte payload limit. This still provides
 * 128 bits of authentication strength — adequate for this use-case.
 *
 * @param device_id   4-byte device identifier
 * @param counter     current rolling code counter
 * @param timestamp   current Unix time from RTC
 * @param out_hmac    output buffer — must be at least 16 bytes
 */
static void compute_hmac(uint32_t device_id, uint32_t counter,
                          uint32_t timestamp, uint8_t *out_hmac) {

    // Message: concatenate device_id || counter || timestamp (12 bytes)
    uint8_t message[12];
    memcpy(message + 0, &device_id,  4);
    memcpy(message + 4, &counter,    4);
    memcpy(message + 8, &timestamp,  4);

    uint8_t full_hmac[32]; // HMAC-SHA256 always produces 32 bytes

    // Initialise mbedTLS HMAC context
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, info, /*use_hmac=*/1);
    mbedtls_md_hmac_starts(&ctx, SECRET_SEED, sizeof(SECRET_SEED));
    mbedtls_md_hmac_update(&ctx, message, sizeof(message));
    mbedtls_md_hmac_finish(&ctx, full_hmac);
    mbedtls_md_free(&ctx);

    // Copy only first 16 bytes into the packet
    memcpy(out_hmac, full_hmac, 16);
}

// ─── Packet Builder ───────────────────────────────────────────────
/**
 * build_packet()
 *
 * Fills an RKEPacket with current device ID, counter, HMAC, and
 * timestamp.  The checksum field is a simple XOR of the preceding
 * 28 bytes so the Slave can quickly discard corrupt radio frames
 * before doing expensive HMAC verification.
 */
static void build_packet(RKEPacket *pkt, uint32_t timestamp) {

    pkt->device_id = DEVICE_ID;
    pkt->counter   = g_counter;
    pkt->timestamp = timestamp;

    // Compute HMAC over (device_id, counter, timestamp)
    compute_hmac(pkt->device_id, pkt->counter, pkt->timestamp, pkt->hmac);

    // XOR checksum over the first 28 bytes (all fields except checksum)
    uint32_t crc = 0;
    const uint8_t *raw = (const uint8_t *)pkt;
    for (int i = 0; i < 28; ++i) {
        crc ^= ((uint32_t)raw[i]) << ((i % 4) * 8);
    }
    pkt->checksum = crc;
}

// ─── NRF24L01 Initialisation ─────────────────────────────────────
/**
 * init_radio()
 *
 * Configures the NRF24L01 as a pure transmitter:
 *   • Channel 108 (2.508 GHz) — outside the 2.4-GHz Wi-Fi band
 *   • 250 kbps — lowest data-rate for maximum link range
 *   • Maximum PA power
 *   • Auto-ACK disabled — Master broadcasts; no reply expected
 *   • Fixed 32-byte payload
 */
static bool init_radio(void) {
    if (!radio.begin()) {
        DBG_PRINTLN("[Radio] NRF24L01 init FAILED — check wiring!");
        return false;
    }

    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF_DATA_RATE);
    radio.setPALevel(RF_PA_LEVEL);
    radio.setPayloadSize(RF_PAYLOAD_SIZE);
    radio.setAutoAck(false);    // No acknowledgement needed
    radio.openWritingPipe(PIPE_ADDRESS);
    radio.stopListening();      // TX mode

    DBG_PRINTLN("[Radio] NRF24L01 ready (TX mode)");
    DBG_PRINTF("[Radio] Channel=%d, DataRate=250kbps, PA=MAX\n", RF_CHANNEL);
    return true;
}

// ─── DS3231 RTC Initialisation ────────────────────────────────────
/**
 * init_rtc()
 *
 * Initialises the DS3231 over I2C.  If the RTC has lost power and
 * reset, it is seeded with the compile-time timestamp as a fallback
 * (accurate enough for the 60-second freshness window).
 */
static bool init_rtc(void) {
    Wire.begin(PIN_SDA, PIN_SCL);
    if (!rtc.begin()) {
        DBG_PRINTLN("[RTC] DS3231 not found — check I2C wiring!");
        return false;
    }

    if (rtc.lostPower()) {
        DBG_PRINTLN("[RTC] RTC lost power — seeding with compile time");
        // F(__DATE__), F(__TIME__) are the build timestamps
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
    delay(500); // Allow Serial Monitor to connect
    Serial.println("\n╔══════════════════════════════╗");
    Serial.println("║  RKE Master — Key Fob Boot   ║");
    Serial.println("╚══════════════════════════════╝");
#endif

    // Initialise peripherals; halt with error blink if critical init fails
    if (!init_rtc()) {
        DBG_PRINTLN("[FATAL] RTC init failed. Halting.");
        while (true) delay(1000);
    }

    nvs_load_counter();

    if (!init_radio()) {
        DBG_PRINTLN("[FATAL] Radio init failed. Halting.");
        while (true) delay(1000);
    }

    DBG_PRINTLN("[Setup] Master ready. Beginning transmission loop.");
}

// ─── Arduino Loop ─────────────────────────────────────────────────
void loop(void) {
    // ── 1. Get current Unix timestamp from RTC ───────────────────
    DateTime now     = rtc.now();
    uint32_t ts_unix = now.unixtime();

    // ── 2. Build the rolling-code packet ─────────────────────────
    RKEPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_packet(&pkt, ts_unix);

#ifdef DEBUG
    DBG_PRINTF("\n[TX] Counter=%u  Timestamp=%u\n", g_counter, ts_unix);
    DBG_PRINT("[TX] HMAC prefix: ");
    for (int i = 0; i < 8; i++) DBG_PRINTF("%02X", pkt.hmac[i]);
    DBG_PRINTLN("...");
#endif

    // ── 3. Transmit packet ───────────────────────────────────────
    // radio.write() returns true on success (ACK or no-ACK timeout).
    // Since Auto-ACK is off, this always returns immediately.
    bool ok = radio.write(&pkt, sizeof(pkt));
    DBG_PRINTF("[TX] Transmit %s\n", ok ? "OK" : "FAIL (check radio)");

    // ── 4. Increment counter and persist to NVS ──────────────────
    // Counter increments even on TX failure so codes are never reused.
    g_counter++;
    nvs_save_counter();

    // ── 5. Light sleep until next transmission window ───────────
    // esp_sleep_enable_timer_wakeup takes microseconds.
    // Light sleep keeps RAM, peripherals, and SPI powered; only CPU
    // clock gates, saving significant current vs. active delay().
    radio.powerDown(); // Put NRF24L01 in standby to save ~12 mA
    esp_sleep_enable_timer_wakeup((uint64_t)TX_INTERVAL_MS * 1000ULL);
    esp_light_sleep_start();
    radio.powerUp();   // Wake NRF24L01 back up before next TX
}
