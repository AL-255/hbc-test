# PPM (Pulse-Position Modulation) Transmitter

Firmware for the **Heltec WiFi LoRa 32 V3** (`HTIT-WB32LA(F)_V3`, ESP32-S3).
Constant-width pulses whose **interval** carries the data (streamed through the
**RMT** peripheral), plus a free-running **1 MHz reference**.

## Output

| Signal | Pin    | Description                                                  |
|--------|--------|--------------------------------------------------------------|
| IO5    | GPIO5  | PPM pulse, 200 ns wide (RMT channel A)                        |
| IO6    | GPIO6  | same polarity as IO5, narrower by 2x dead time (RMT channel B, synced) |
| IO7    | GPIO7  | free-running 1 MHz, 50% square wave (LEDC)                    |

* **IO5 width:** 200 ns, constant (16 RMT ticks @ 80 MHz)
* **IO6 width:** `200 ns - 2 x DEAD_TICKS x 12.5 ns`, **centered inside** IO5's
  pulse — same polarity, inset by the dead time on each edge (default
  `DEAD_TICKS = 2` -> 25 ns inset, IO6 = 150 ns)
* **Interval:** `400 ns + position x 12.5 ns`
* **Payload:** value `0..127`, `position = value >> 1` -> 64 positions
  (2:1 mapping), interval **400 ns .. 1187.5 ns** (inside the 400-1200 ns window)
* **Burst:** the full 0..127 sweep = 128 symbols per RMT channel, fired as one
  synchronized burst; the IO7 PWM runs continuously throughout

```
IO5  ___|‾‾‾‾‾‾‾‾|________________|‾‾‾‾‾‾‾‾|___   200 ns
IO6  _____|‾‾‾‾|____________________|‾‾‾‾|_____   200 ns - 2*DT, centered
        D ^    ^ D                                dead-time inset each edge
IO7  ‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|    1 MHz, 50%
```

## Why RMT (and why 12.5 ns, not 6.25 ns)

The shortest interval in the sweep is **400 ns**. A per-pulse timer ISR (which
is the only way to get MCPWM's 6.25 ns resolution) would have to enter, run, and
return inside that 400 ns, 128 times back to back — the driver's ISR path alone
is ~1 us, so the tight pulses would overrun and the timing would desync. RMT
sidesteps that: it walks a pre-built symbol buffer in hardware. Even **without
DMA** the CPU only refills a ~48-symbol block every ~20-40 us (not per pulse),
so the pulse rate is bounded by hardware, not interrupt latency.

The cost is resolution: on the ESP32-S3 **RMT tops out at 80 MHz**, so the
timing grid is **12.5 ns**. 6.25 ns edge placement is not reachable by any DMA
path on this chip either. At a 12.5 ns grid the 400-1200 ns window holds 64
distinct positions, so the 128-value payload maps 2:1.

## How the frame is built

Each value becomes one RMT symbol on each leg; the two legs run on **separate
RMT channels started by a sync manager** so they share a clock edge (a single
DMA channel is the S3's limit, and IO6 needs its own timing to be narrower).

```c
for (i = 0; i < 128; i++) {
    s_payload[i] = i;
    I = 32 + (i >> 1);                 // interval in 12.5 ns ticks

    // IO5 : 200 ns HIGH, then the gap
    a.duration0 = 16;          a.level0 = 1;
    a.duration1 = I - 16;      a.level1 = 0;

    // IO6 : same polarity, narrower -- HIGH (16 - 2*D), inset by D each edge.
    //       {LOW gap up to the pulse, HIGH narrow body}; gap = D on the first
    //       symbol, prev_interval - 16 + 2*D afterwards.
    b.duration0 = gap;         b.level0 = 0;
    b.duration1 = 16 - 2*D;    b.level1 = 1;
}
```

A **copy encoder** per channel streams the arrays; `rmt_sync_reset()` re-aligns
the pair before each burst. Because IO6 is its own timed channel, its pulse sits
`DEAD_TICKS x 12.5 ns` inside IO5's on each edge. IO5 is byte-for-byte the same
waveform as the plain PPM. IO7's 1 MHz / 50% reference is a separate **LEDC**
channel that free-runs independent of the bursts.

## Build & flash

On Windows, double-click `build_flash.bat` and enter the Heltec board's COM
port, or run it from a terminal:

```bat
build_flash.bat COM7
build_flash.bat --build-only --no-pause
```

The batch file loads `C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1`
for ESP-IDF at `C:\esp`, selects ESP32-S3, and builds before flashing.
It keeps the result visible unless the second argument is `--no-pause`.

```bash
./flash.sh -m          # build, auto-detect port, flash, open monitor
./flash.sh -b          # build only
./flash.sh -p /dev/ttyUSB0
```

(Uses ESP-IDF from `~/.espressif/v6.0.1/esp-idf`; override with `IDF_PATH=...`.)

## Tuning (top of `main/main.c`)

| Macro            | Meaning                              |
|------------------|--------------------------------------|
| `PULSE_TICKS`    | IO5 pulse width in 12.5 ns ticks (16 = 200 ns) |
| `BASE_TICKS`     | minimum interval (32 = 400 ns)       |
| `DEAD_TICKS`     | IO6 inset per edge, 1..7 (12.5..87.5 ns); 2 = 25 ns |
| `N_VALUES`       | payload length / sweep size          |
| `OUT_A/B_GPIO`   | IO5 / IO6 pins                       |
| `PWM_GPIO` / `PWM_FREQ_HZ` | IO7 reference pin / rate (1 MHz) |
| `INTER_BURST_US` | gap between repeated demo bursts (us) |

The demo repeats the burst every `INTER_BURST_US` (100 us) so it is easy to
catch on a scope; the gap is a busy-wait since it is below one FreeRTOS tick.
For a true one-shot, call `rmt_transmit()` once instead of in the loop.
