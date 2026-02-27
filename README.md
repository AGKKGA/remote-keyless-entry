# Remote Keyless Entry (RKE) System

> **Portfolio project** — ESP32 + NRF24L01 proximity-based door unlocker with rolling-code security.

---

## Repository Layout

```
remote-keyless-entry/
├── shared_config.h        ← Shared constants (MUST be identical on both devices)
├── master/
│   └── master.ino         ← Key fob firmware (TX)
├── slave/
│   └── slave.ino          ← Door unit firmware (RX)
└── docs/
    └── wiring.md          ← Full wiring tables for all 6 connections
```

---

## Prerequisites

### Arduino IDE Setup
1. Install **Arduino IDE 2.x** (recommended).
2. Add ESP32 board support:
   - *File → Preferences → Additional Board Manager URLs*
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - *Tools → Board Manager* → search **esp32** → install **esp32 by Espressif Systems** (v2.x or later).
3. Select board: **ESP32 Dev Module**.

### Required Libraries (install via *Tools → Manage Libraries…*)

| Library | Device | Purpose |
|---|---|---|
| **RF24** by nRF24 | Master + Slave | NRF24L01 radio driver |
| **RTClib** by Adafruit | Master + Slave | DS3231 RTC driver |
| **ESP32Servo** by Kevin Harrington | Master only | Servo on ESP32 |
| **Crypto** by Rhys Weatherley | Slave only | SHA256 for manual HMAC-SHA256 on Nano |

> - `Preferences.h` (NVS) and `mbedtls/md.h` (HMAC) are built into the **ESP32** Arduino core — Master only.
> - `EEPROM.h` and `Servo.h` are bundled with the **Arduino AVR** core — Slave only, no install needed.

---

## Part 1 — Pairing the Two Devices (Shared Secret Setup)

Both devices are "paired" at flash time by sharing the same `shared_config.h`. There is no over-the-air pairing — this is intentional and more secure.

**Steps:**

1. Open `shared_config.h` in a text editor.

2. **Generate a random 32-byte secret seed.** Use any cryptographically secure random source, for example:

   ```bash
   # Linux / macOS / WSL
   python3 -c "import secrets; print(', '.join(hex(b) for b in secrets.token_bytes(32)))"
   ```

   Paste the output into the `SECRET_SEED` array.

3. **Set a unique `DEVICE_ID`** (4-byte hex value). Pick anything; it just needs to be non-zero and consistent.

4. Optionally change `PIPE_ADDRESS` if you want to prevent interference with another RKE unit in the same area.

5. Copy the **identical** `shared_config.h` to both the `master/` and `slave/` sketch folders.

6. Flash `master.ino` to the key-fob ESP32 and `slave.ino` to the door-unit ESP32 (see flashing steps below).

> [!IMPORTANT]
> Once deployed, the `SECRET_SEED` must never be transmitted, logged to Serial in production, or stored in plaintext anywhere accessible. In this project the `DEBUG` flag prints HMAC prefixes — **disable DEBUG before production use** by commenting out `#define DEBUG` in `shared_config.h`.

---

## Part 2 — Flashing

### Master (Key Fob)
```
1. Open master/master.ino in Arduino IDE.
2. Place shared_config.h in the same folder as master.ino.
3. Select: Tools → Board → ESP32 Dev Module
4. Select: Tools → Port → (your Master ESP32 COM port)
5. Click Upload.
6. Open Serial Monitor (115200 baud) to see boot log and TX events.
```

### Slave (Door Unit — Arduino Nano)
```
1. Open slave/slave.ino in Arduino IDE.
2. Place shared_config.h in the same folder as slave.ino.
3. Select: Tools → Board → Arduino Nano
4. Select: Tools → Processor → ATmega328P
   (for older bootloader Nanos: select "ATmega328P (Old Bootloader)")
5. Select the correct COM port for the Slave Nano.
6. Click Upload.
7. Open Serial Monitor (115200 baud) to see RX validation log.
```

> [!TIP]
> **First boot**: The Nano's EEPROM is blank. The firmware detects this via a magic word and initialises the counter to 0 automatically — no manual setup needed.

> [!TIP]
> You can use two separate Arduino IDE instances, each with its own Serial Monitor, to observe both sides simultaneously.

---

## Part 3 — Testing Rolling Code Validity

**Expected normal operation:**

1. Power on Slave. Serial Monitor should show:
   ```
   RKE Slave — Door Unit Boot
   [RTC] Current UTC: 2026-02-27 21:46:00
   [NVS] Loaded last counter = 0
   [Radio] NRF24L01 ready (RX mode)
   [Setup] Slave ready. Listening for RKE packets...
   ```

2. Power on Master. After 500 ms you should see on the Slave:
   ```
   [RX] Packet received | DeviceID=0xDEADBEEF Counter=1 TS=1772117160
   [RX] Packet VALID - Unlocking door!
   [SERVO] Unlocking door...
   [SERVO] Door re-locked.
   [NVS] Saved last counter = 1
   ```
   And the servo motor rotates 90°, holds for 5 seconds, returns to 0°.

3. Subsequent packets (counter 2, 3, 4 …) are also accepted because the Slave advances its window.

**Test counter desync recovery:**

1. Power off Master. Manually edit NVS on Master:
   - In `master.ino`, temporarily change `nvs_load_counter()` to override: `g_counter = stored_value + 15;`
   - Re-flash, power on.
   - Slave should accept the packet (within ±20 window) and jump its own counter to match.

2. Jump by more than 20 (e.g., set `g_counter = stored_value + 25`):
   - Slave Serial Monitor should show: `[RX] REJECTED — reason: COUNTER_OUT_OF_WINDOW`
   - This simulates the fob being pressed many times in a Faraday cage.

---

## Part 4 — Simulating a Replay Attack

A **replay attack** is when an adversary captures a valid packet and re-transmits it later.

### Simulation Setup

You need a second ESP32 or a radio sniffer that can replay raw NRF24L01 packets. A simple replay attacker sketch:

```cpp
// replay_attacker.ino — FOR TESTING ONLY
// Configure NRF24L01 on same channel/address as the system.
// Capture one packet, store it, re-transmit it multiple times.

#include <SPI.h>
#include <RF24.h>
// ... (same channel 108, same PIPE_ADDRESS, RX mode)

uint8_t captured[32];
bool    captured_valid = false;

void loop() {
    if (!captured_valid && radio.available()) {
        radio.read(captured, 32);
        captured_valid = true;
        Serial.println("Packet captured! Replaying...");
        radio.stopListening();
    }
    if (captured_valid) {
        radio.write(captured, 32);  // Re-transmit captured packet
        delay(500);
    }
}
```

### Expected Result

**First replay (if within timestamp window and counter window):**
- The Slave **rejects** it because the counter is already in `g_replay_cache`.
- Serial Monitor shows: `[RX] REJECTED — reason: REPLAY_DETECTED`

**After 60 seconds (timestamp freshness expires):**
- Even if the attacker retransmits: `[RX] REJECTED — reason: TIMESTAMP_STALE`

**After the counter window has moved past the captured counter:**
- `[RX] REJECTED — reason: COUNTER_OUT_OF_WINDOW`

> [!NOTE]
> The system provides **three independent layers of replay protection**:
> 1. **Anti-replay cache** — consumed counters are blacklisted immediately.
> 2. **60-second timestamp window** — stale packets are rejected even if the counter is valid.
> 3. **Counter window** — once the Slave counter advances far enough, old codes are permanently rejected.

---

## Security Summary

| Threat | Mitigation |
|---|---|
| Passive sniffing | HMAC-SHA256 — code is unguessable without secret seed |
| Replay attack | Anti-replay cache + 60 s timestamp window |
| Brute-force counter | 2³² counter space + HMAC covers all combinations |
| Cloning attack | SECRET_SEED never transmitted; only in flash |
| MITM / code substitution | HMAC validation; any bit flip changes the HMAC |
| Desync / lockout | ±20 counter window with counter jump-ahead on valid packet |