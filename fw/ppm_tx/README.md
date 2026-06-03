# PPM (Pulse-Position Modulation) Transmitter

Firmware for the **Heltec WiFi LoRa 32 V3** (`HTIT-WB32LA(F)_V3`, ESP32-S3).
Constant-width pulses whose **interval** carries the data, streamed as a single
**DMA burst** through the **RMT** peripheral.

## Output

| Signal | Pin    | Description                                  |
|--------|--------|----------------------------------------------|
| OUT+   | GPIO5  | RMT TX                                        |
| OUT-   | GPIO6  | exact inverse of OUT+ (zero skew, see below)  |

* **Pulse width:** 200 ns, constant (16 RMT ticks @ 80 MHz)
* **Interval:** `400 ns + position x 12.5 ns`
* **Payload:** value `0..127`, `position = value >> 1` -> 64 positions
  (2:1 mapping), interval **400 ns .. 1187.5 ns** (inside the 400-1200 ns window)
* **Burst:** the full 0..127 sweep = 128 symbols, fired as one DMA transfer

## Why RMT + DMA (and why 12.5 ns, not 6.25 ns)

The shortest interval in the sweep is **400 ns**. A per-pulse timer ISR (which
is the only way to get MCPWM's 6.25 ns resolution) would have to enter, run, and
return inside that 400 ns, 128 times back to back — the driver's ISR path alone
is ~1 us, so the tight pulses would overrun and the timing would desync. RMT
sidesteps the CPU entirely: it walks a pre-built symbol buffer over DMA, so the
pulse rate is bounded by hardware, not interrupt latency.

The cost is resolution: on the ESP32-S3 **RMT tops out at 80 MHz** (every
DMA-capable peripheral does — SPI, LCD, I2S too), so the timing grid is
**12.5 ns**. 6.25 ns edge placement is not reachable by any DMA path on this
chip. At a 12.5 ns grid the 400-1200 ns window holds 64 distinct positions, so
the 128-value payload maps 2:1 (pairs of values share a position).

## How the frame is built

```c
// payload buffer: the 0..127 sweep
for (i = 0; i < 128; i++) s_payload[i] = i;

// each value -> one RMT symbol: 200 ns HIGH, then a gap
symbol.duration0 = 16;            level0 = 1;     // 200 ns pulse
symbol.duration1 = 16 + (v >> 1); level1 = 0;     // interval - pulse
```

The array of symbols is handed to a **copy encoder** and `rmt_transmit()`s as
one burst; the **complementary OUT-** is produced by mirroring the same RMT
output signal onto GPIO6 through the GPIO matrix with inversion — both legs come
from one internal signal, so there is no inter-channel skew.

## Build & flash

```bash
./flash.sh -m          # build, auto-detect port, flash, open monitor
./flash.sh -b          # build only
./flash.sh -p /dev/ttyUSB0
```

(Uses ESP-IDF from `~/.espressif/v6.0.1/esp-idf`; override with `IDF_PATH=...`.)

## Tuning (top of `main/main.c`)

| Macro            | Meaning                              |
|------------------|--------------------------------------|
| `PULSE_TICKS`    | pulse width in 12.5 ns ticks (16 = 200 ns) |
| `BASE_TICKS`     | minimum interval (32 = 400 ns)       |
| `N_VALUES`       | payload length / sweep size          |
| `OUT_A/B_GPIO`   | OUT+ / OUT- pins                     |
| `INTER_BURST_MS` | gap between repeated demo bursts     |

The demo repeats the burst every `INTER_BURST_MS` so it is easy to catch on a
scope; for a true one-shot, call `rmt_transmit()` once instead of in the loop.
