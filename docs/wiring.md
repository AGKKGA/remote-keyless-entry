# RKE System — Wiring Reference

All connections use **ESP32 DevKit v1** (38-pin).

> [!WARNING]
> **Pins to avoid:**
> - **GPIO 6–11** — connected to internal flash (SPI flash bus). Never use.
> - **GPIO 34, 35, 36 (VP), 39 (VN)** — input-only, no internal pull-up/pull-down.
> - **GPIO 0, 2, 12, 15** — strapping pins. Safe at runtime but avoid pull-downs during boot.
> - **ADC2 pins (GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27)** — ADC2 is disabled when Wi-Fi is active. Not relevant here (Wi-Fi unused), but noted for awareness.

---

## 1 · Master — ESP32 → NRF24L01 (SPI)

Power the NRF24L01 from the **3.3 V** rail only. Never connect to 5 V — the module is not 5 V tolerant.
Add a **10 µF electrolytic + 100 nF ceramic capacitor** across VCC/GND close to the NRF24L01 to suppress the large current spikes the module draws on TX.

| NRF24L01 Pin | ESP32 GPIO | Wire colour (suggested) | Notes |
|---|---|---|---|
| VCC | 3.3 V | Red | Max 250 mA peak — use bulk cap |
| GND | GND | Black | Common ground |
| CE | GPIO 4 | Orange | Chip Enable — driven by RF24 lib |
| CSN | GPIO 5 | Yellow | SPI Chip Select (active LOW) |
| SCK | GPIO 18 | Blue | VSPI Clock |
| MOSI | GPIO 23 | Green | VSPI Master-Out |
| MISO | GPIO 19 | White | VSPI Master-In |
| IRQ | — | — | Not connected (polling used) |

---

## 2 · Master — ESP32 → DS3231 (I2C)

The DS3231 module typically includes on-board pull-up resistors (4.7 kΩ) on SDA/SCL. If not, add them externally to 3.3 V.

| DS3231 Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3.3 V | Module accepts 3.3 V–5.5 V; use 3.3 V here |
| GND | GND | Common ground |
| SDA | GPIO 21 | ESP32 default Wire SDA |
| SCL | GPIO 22 | ESP32 default Wire SCL |
| SQW | — | Not connected |
| 32K | — | Not connected |

---

## 3 · Master — ESP32 Power (18650 via TP4056 + 3.3 V Regulator)

Use a **TP4056 module with protection** (the version with the DW01A + FS8205A dual-chip). This provides over-charge, over-discharge, and short-circuit protection.
Use an **AMS1117-3.3** or **HT7333** LDO regulator between the TP4056 OUT+ and the ESP32 VIN pin.

```
18650 Cell
  (+) ──→ TP4056 BAT+ ──→ TP4056 OUT+ ──→ LDO IN ──→ LDO OUT (3.3 V) ──→ ESP32 3.3V pin
  (-) ──→ TP4056 BAT- ──→ TP4056 OUT- ──→ LDO GND ──→ ESP32 GND
```

| TP4056 Pin | Connection | Notes |
|---|---|---|
| IN+ | USB 5 V (micro-USB port) | Charging input |
| IN- | USB GND | |
| BAT+ | 18650 positive | |
| BAT- | 18650 negative | |
| OUT+ | LDO input | Protected output — use for load |
| OUT- | GND rail | |

| LDO Pin | Connection |
|---|---|
| IN | TP4056 OUT+ |
| OUT | ESP32 3.3 V pin or VIN (if using 3.3 V LDO) |
| GND | Common GND |

> [!TIP]
> If the NRF24L01 causes brownouts during TX, place a dedicated **100 µF capacitor** across the LDO output. NRF24L01 TX bursts can draw up to 115 mA transiently.

---

## 4 · Slave — ESP32 → NRF24L01 (SPI)

Identical to the Master wiring (Table 1). The Slave uses the same VSPI pins.

| NRF24L01 Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3.3 V | With bypass caps (10 µF + 100 nF) |
| GND | GND | |
| CE | GPIO 4 | |
| CSN | GPIO 5 | |
| SCK | GPIO 18 | |
| MOSI | GPIO 23 | |
| MISO | GPIO 19 | |
| IRQ | — | Not connected |

---

## 5 · Slave — ESP32 → DS3231 (I2C)

Identical to the Master I2C wiring (Table 2).

| DS3231 Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| SDA | GPIO 21 | |
| SCL | GPIO 22 | |

---

## 6 · Slave — ESP32 → Servo Motor (PWM)

> [!CAUTION]
> **Do NOT power the servo from the ESP32 3.3 V rail.** A standard SG90 servo draws ~300 mA under load; a medium servo (MG996R) draws up to 1 A stall. This will either brown-out the ESP32 or destroy the 3.3 V LDO.
> Use an **external 5 V supply** (phone charger, USB hub, 5 V buck converter from the wall adapter) with its GND tied to the ESP32 GND.

| Servo Wire | Connection | Notes |
|---|---|---|
| Signal (Orange/Yellow) | GPIO 13 | PWM output from ESP32. GPIO 13 is safe at boot |
| Power (Red) | External 5 V supply | NOT ESP32 3.3 V |
| Ground (Brown/Black) | Common GND | Tied to ESP32 GND and 5 V supply GND |

**Servo PWM parameters** (set in `slave.ino`):
- Pulse range: 500 µs (0°) → 2400 µs (180°)
- Locked position: 0° (500 µs)
- Unlocked position: 90° (1450 µs ≈ midpoint)
- Frequency: 50 Hz (standard hobby servo)

---

## Quick-Reference GPIO Map

| GPIO | Master Role | Slave Role |
|---|---|---|
| 4 | NRF24 CE | NRF24 CE |
| 5 | NRF24 CSN | NRF24 CSN |
| 13 | — | Servo PWM |
| 18 | SPI SCK | SPI SCK |
| 19 | SPI MISO | SPI MISO |
| 21 | I2C SDA | I2C SDA |
| 22 | I2C SCL | I2C SCL |
| 23 | SPI MOSI | SPI MOSI |
