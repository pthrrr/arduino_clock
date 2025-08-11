# Wi‑Fi LED Matrix Clock (WeMos D1 mini + MAX7219)

A small Wi‑Fi clock for a 4‑module MAX7219 LED matrix (8×32) driven by a WeMos D1 mini (ESP8266).

Features
- NTP time sync with Berlin timezone (CET/CEST automatic DST)
- Auto‑brightness via light sensor on A0 (smoothed)
- Wi‑Fi credentials read from a local `secrets.h` (see `secrets.h.example`)

Hardware
- WeMos D1 mini (ESP8266)
- 4‑in‑1 MAX7219 LED matrix (“FC‑16” type)
- LDR + resistor divider to A0

Wiring
- Matrix VCC → 3V3 (works; keep brightness low) or 5V (preferred), GND → G
- Matrix DIN → D7, CS → D4, CLK → D5
- LDR divider output → A0 (typical: LDR→3V3, 10k→GND, junction→A0)

Notes
- If text looks mirrored/jumbled, change `HARDWARE_TYPE` in the sketch between `FC16_HW` and `ICSTATION_HW`.
- On 3.3V, brightness is clamped in code for stability.

## Arduino IDE Setup (2.x)

- Add ESP8266 boards (Board Manager)  
   - File → Preferences → Additional Boards Manager URLs:  
     `http://arduino.esp8266.com/stable/package_esp8266com_index.json`  
   - Tools → Board → Boards Manager → search “esp8266” → install “esp8266 by ESP8266 Community”  
   - Select board: Tools → Board → ESP8266 Boards → “LOLIN(WEMOS) D1 R2 & mini”

- Install libraries (Library Manager)  
   - Install “MD_Parola” (will also install “MD_MAX72XX”; if not, install it manually)  
   - ESP8266 core provides ESP8266WiFi and time.h
