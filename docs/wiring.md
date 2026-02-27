# RKE System — Wiring Reference

**Master** uses **ESP32 DevKit v1** (38-pin). **Slave** uses **Arduino Nano** (ATmega328P).

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

## 4 · Slave — Arduino Nano → NRF24L01 (SPI)

> [!CAUTION]
> **Nano's onboard 3.3 V pin is limited to ~50 mA** from its AMS1117 regulator. The NRF24L01+PA external-antenna module can draw **up to 115 mA on TX** — this will brown-out the Nano.
> **Use a separate AMS1117-3.3 breakout (or LD1117-3.3) powered from the Nano's 5 V pin** to supply the NRF24L01 independently. The basic non-PA NRF24L01 (12 mA max) is safe from the Nano's 3.3 V pin.

Add a **10 µF electrolytic + 100 nF ceramic capacitor** across VCC/GND close to the module.

| NRF24L01 Pin | Nano Pin | Wire colour (suggested) | Notes |
|---|---|---|---|
| VCC | External 3.3 V⁽¹⁾ | Red | **NOT** Nano's 3.3 V pin for PA module |
| GND | GND | Black | Common ground |
| CE | D9 | Orange | Chip Enable |
| CSN | D10 | Yellow | SPI Chip Select (active LOW) |
| SCK | D13 | Blue | Hardware SPI clock (also drives built-in LED) |
| MOSI | D11 | Green | Hardware SPI Master-Out |
| MISO | D12 | White | Hardware SPI Master-In |
| IRQ | — | — | Not connected (polling used) |

⁽¹⁾ External 3.3 V: wire an AMS1117-3.3 IN to Nano's 5 V pin, OUT to NRF24L01 VCC.

---

## 5 · Slave — Arduino Nano → DS3231 (I2C)

Nano's I2C bus is **fixed** to A4 (SDA) and A5 (SCL) in hardware — `Wire.begin()` uses them automatically without passing pin arguments.

The DS3231 module's onboard 4.7 kΩ pull-ups are sufficient. Power from 5 V for reliable operation.

| DS3231 Pin | Nano Pin | Notes |
|---|---|---|
| VCC | 5 V | Module is 3.3 V–5.5 V tolerant; 5 V preferred on Nano |
| GND | GND | Common ground |
| SDA | A4 | Fixed hardware I2C SDA on Nano |
| SCL | A5 | Fixed hardware I2C SCL on Nano |
| SQW | — | Not connected |
| 32K | — | Not connected |

---

## 6 · Slave — Arduino Nano → Servo Motor (PWM)

> [!CAUTION]
> **Do NOT power the servo from the Nano's 5 V pin when sourced from USB.** USB provides ~500 mA total; an SG90 servo draws ~300 mA under load and the Nano itself needs ~50 mA, leaving little margin. A medium servo (MG996R) can stall at 1 A+.
> Use a **dedicated external 5 V supply** (USB charger, bench supply, or 5 V buck converter) with its GND tied to the Nano's GND.

| Servo Wire | Nano Pin | Notes |
|---|---|---|
| Signal (Orange/Yellow) | D6 | PWM-capable at 490 Hz. Avoids SPI (D11–D13) and I2C (A4/A5) pins |
| Power (Red) | External 5 V | NOT Nano's 5 V pin under USB power |
| Ground (Brown/Black) | Common GND | Tied to Nano GND and external 5 V GND |

**Servo PWM parameters** (set in `slave.ino`):
- Pulse range: 500 µs (0°) → 2400 µs (180°)
- Locked position: 0° (500 µs)
- Unlocked position: 90° (~1450 µs)
- Frequency: 50 Hz (standard hobby servo)

---

## Quick-Reference Pin Map

| Signal | Master (ESP32) | Slave (Nano) |
|---|---|---|
| NRF24 CE | GPIO 4 | D9 |
| NRF24 CSN | GPIO 5 | D10 |
| SPI SCK | GPIO 18 | D13 |
| SPI MOSI | GPIO 23 | D11 |
| SPI MISO | GPIO 19 | D12 |
| I2C SDA | GPIO 21 | A4 (fixed) |
| I2C SCL | GPIO 22 | A5 (fixed) |
| Servo PWM | — | D6 |
