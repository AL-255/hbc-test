# 1 MHz Complementary (Differential) Pulse Generator

Firmware for the **Heltec WiFi LoRa 32 V3** (`HTIT-WB32LA(F)_V3`, ESP32-S3).
Generates a hardware-timed complementary pulse pair using the ESP32-S3 **MCPWM**
peripheral.

## Output

| Signal | Pin    | Description                         |
|--------|--------|-------------------------------------|
| OUT+   | GPIO5  | true pulse, HIGH for 200 ns         |
| OUT-   | GPIO6  | exact inverse of OUT+               |

* **Repetition rate:** 1.000 MHz (1 µs period)
* **Pulse high-time:** 200 ns (32 MCPWM ticks @ 160 MHz, 6.25 ns/tick).
  Resolution is 6.25 ns; set `PULSE_TICKS = 1` for the narrowest pulse the
  peripheral can emit.

A single MCPWM timer drives both generators, so the two outputs are true
inverses with **no firmware skew** — they switch on the same hardware events.

### Why these pins
GPIO5/GPIO6 are adjacent, broken out on the V3 header, and free of the LoRa
radio (8–14), OLED (17/18/21), battery ADC (1/37), strapping pins (0/3/45/46),
USB/UART (19/20/43/44) and internal flash/PSRAM (26–32). Adjacent pins let you
route a matched, length-controlled differential pair. `GPIO6`/`GPIO7` is an
equally good alternate. Change `GEN_A_GPIO` / `GEN_B_GPIO` in the source to move
them.

> Pulse width is `PULSE_TICKS x 6.25 ns`. Below ~5 ticks the GPIO's own slew
> rate dominates and the pulse may not reach the full rail. Keep both traces
> short, equal length, and **terminate the pair at the receiver**.

## Build & flash (ESP-IDF v5.x)

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor   # adjust the serial port
```

## Build & flash (Arduino alternative)

Open `arduino/diff_pulse_1mhz.ino` in the Arduino IDE with **Arduino-ESP32 v3.x**,
select board **"Heltec WiFi LoRa 32(V3)"**, and upload. Same pins and timing.

## Tuning

Edit the `#define`s at the top of `main/main.c` (or the `.ino`):

| Macro          | Meaning                          | Effect                                  |
|----------------|----------------------------------|-----------------------------------------|
| `PERIOD_TICKS` | timer ticks per cycle            | rep rate = `TIMER_RES_HZ / PERIOD_TICKS`|
| `PULSE_TICKS`  | high-time in ticks (6.25 ns each)| pulse width                             |
| `GEN_A/B_GPIO` | output pins                      | physical OUT+/OUT- pins                 |
```
