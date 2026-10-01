# XIAO ESP32-S3 PPM transmitter

Port of `../ppm_tx` for the **Seeed Studio XIAO ESP32-S3**, with an independent
configurable PWM output. Two synchronized RMT channels generate same-polarity PPM pulses,
with the narrower pulse centered inside the wider pulse.

## Outputs

| Signal | XIAO pin | ESP32-S3 GPIO | Behavior |
|--------|----------|---------------|----------|
| PWM | D0 | GPIO1 | 100 kHz, 50% duty (5 us HIGH / 5 us LOW) |
| PPM A | D4 | GPIO5 | Outer 200 ns pulse, idle low |
| PPM B | D3 | GPIO4 | Same polarity, centered 150 ns pulse, idle low |

The board labels determine the pin mapping: D3/D4 are GPIO4/5, as listed in
the [Seeed pinout](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).
GPIO5/6 would instead be D4/D5.

* PPM A pulse width: **200 ns** (`PULSE_TICKS = 16` at 80 MHz), unchanged.
* PPM B has the **same polarity**, with its HIGH pulse centered inside A:
  `PULSE_TICKS - 2 * DEAD_TICKS`, or **150 ns** with the defaults.
* Dead time is an inset at **each edge** of A's pulse: **10% of the configured
  pulse width**, rounded up to the next 12.5 ns tick. The requested 20 ns
  becomes **25 ns** (`DEAD_TICKS = 2`). B rises 25 ns after A and falls 25 ns
  before A. During these margins A is HIGH and B is LOW.
* A rising-edge interval: `400 ns + (value >> 1) * 12.5 ns`, from **400 ns to
  1187.5 ns**. The inset does not extend the interval.
* The 128-value sweep is pre-rendered. A uses the S3's single DMA TX
  channel; B uses three hardware memory blocks (144 symbols), enough for
  its 129-symbol frame plus EOF. No mid-frame refill interrupt is needed.
* The RMT sync manager starts both channels together and is reset before each
  burst. Both outputs remain LOW during the interrupt-timed inter-burst gap
  (at least 100 us, plus interrupt/wakeup and next-burst setup latency).
* D0 runs independently of PPM, currently at **100 kHz, 50% duty**.
  `PWM_DUTY_PERCENT = 100` holds it HIGH;
  `0` holds it LOW. For `1..99`, LEDC generates continuous PWM at
  `PWM_FREQ_HZ` (currently 100 kHz), including between PPM bursts.
  PWM phase is not synchronized to PPM.

For each payload value, the sequence is:

```text
A HIGH, B LOW for DEAD_TICKS
both HIGH for PULSE_TICKS - 2 * DEAD_TICKS
A HIGH, B LOW for DEAD_TICKS
both LOW for BASE_TICKS + (value >> 1) - PULSE_TICKS
```

Use a scope to check D0's frequency/duty and the nested PPM pulses
after flashing. Compilation alone does not verify physical output timing.

## Power consumption

The transmitter task blocks throughout each hardware-generated frame and its
inter-burst gap. RMT/GDMA completion interrupts start a one-shot GPTimer after
both channels finish. The timer interrupt wakes the task after `INTER_BURST_US`.
There is no polling, application busy-wait delay, or per-pulse CPU interrupt.
A's complete frame and end marker fit in one DMA descriptor, avoiding a
mid-frame descriptor interrupt; B's complete frame fits in RMT hardware memory.

ESP-IDF's idle task executes the CPU's **WAITI** (wait for interrupt) instruction
while the transmitter task is blocked. This is CPU idle sleep, not chip
Light-sleep or Deep-sleep: those modes would stop the 80 MHz APB-driven
RMT/DMA waveform. The APB/peripheral clocks stay running while the CPU waits.
The CPU runs at a fixed **80 MHz**, with dynamic frequency scaling disabled,
and only CPU0 is started. RMT explicitly uses the fixed **80 MHz APB** clock;
D0 PWM and the gap timer use the **40 MHz crystal**. Pulse timing is generated
by peripherals, without changing CPU frequency for each burst.

The minimal build excludes Wi-Fi, Bluetooth/BLE, and their RF PHY driver;
these radios are never initialized. The previous firmware also did not start
the radios, so removing these drivers does not imply additional radio current
savings. USB Serial/JTAG remains available for flashing and monitoring.

Status logging is limited to one report every five seconds. Reports show idle
task runtime share and idle-hook entries leading to WAITI. Idle task runtime
includes some interrupt overhead; it is not an exact measurement of time spent
in WAITI or of current consumption. Measure supply current and scope the outputs
to verify the physical power reduction and pulse timing on your board.

Power-related build settings are in `sdkconfig.defaults`. When upgrading an
existing build on another machine, back up and regenerate its ignored
`sdkconfig` so these defaults take effect. Compilation checks that CPU frequency
is 80 MHz, single-core mode is selected, and dynamic power management is off.

## Build on Windows

For a one-step build and flash, connect the board and double-click
`build_flash.bat` in this directory. It loads the installed ESP-IDF environment,
builds the firmware, and flashes **COM8** by default. The window stays open so
you can read the result. To use another port from a terminal:

```bat
build_flash.bat COM7
```

Use `build_flash.bat COM8 --no-pause` for terminal or unattended use. A failed
build prevents flashing, and the script returns a nonzero exit code on failure.
It runs from its own directory regardless of where you launch it.

With the installed ESP-IDF toolchain at `C:\esp`:

```powershell
cd C:\Users\yukidama\github\hbc-test\fw\ppm_tx_xiao
$env:PYTHONUTF8 = "1"
. C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1
idf.py build
```

The target defaults to ESP32-S3. Defaults select 8 MB flash and the native USB
Serial/JTAG console. PSRAM is not required or enabled for these waveform buffers.
Build artifacts are in `build/`, including `ppm_tx_xiao.bin`.

## Flash and monitor

Connect the XIAO using a USB data cable and replace `COM7` with its serial port:

```powershell
idf.py -p COM7 flash monitor
```

If the board cannot enter download mode automatically, hold BOOT while
connecting USB (or press RESET while holding BOOT), then release BOOT and retry.
Exit the monitor with Ctrl+].

On other hosts, activate your ESP-IDF environment, then run `idf.py build`
and `idf.py -p <port> flash monitor` from this directory.

## Tuning

`main/main.c` defines `PWM_GPIO`, `PWM_FREQ_HZ`, `PWM_DUTY_PERCENT`, `OUT_A_GPIO`, `OUT_B_GPIO`,
`PULSE_TICKS`, `DEAD_TIME_PERCENT`, `BASE_TICKS`, `N_VALUES`, and `INTER_BURST_US`.
Dead time is computed from `PULSE_TICKS`; changing the pulse-width parameter
automatically changes the inset. The inset on both edges must leave a positive
B pulse (`2 * DEAD_TICKS < PULSE_TICKS`). The frame size is limited to 128 values so
the complete B waveform fits in hardware RAM.
PWM frequency and duty are separate parameters. For example, set
`PWM_FREQ_HZ = 100000` and `PWM_DUTY_PERCENT = 50` for 100 kHz at 50% duty.
At 0% or 100%, GPIO drives a constant level and frequency is unused.
Intermediate duties are rounded to the timer's available resolution, chosen
automatically from the board's 40 MHz crystal and requested frequency.
