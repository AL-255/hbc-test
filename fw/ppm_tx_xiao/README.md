# XIAO ESP32-S3 PPM transmitter

Port of `../ppm_tx` for the **Seeed Studio XIAO ESP32-S3**, with an independent
configurable PWM output. Two synchronized RMT channels generate PPM phases
with a both-LOW dead-time gap at each handoff.

## Outputs

| Signal | XIAO pin | ESP32-S3 GPIO | Behavior |
|--------|----------|---------------|----------|
| PWM | D0 | GPIO1 | 100% duty: constant HIGH (nominal 3.3 V) |
| PPM OUT+ | D4 | GPIO5 | RMT TX, idle low |
| PPM OUT- | D3 | GPIO4 | Complementary phase with dead time, idle low |

The board labels determine the pin mapping: D3/D4 are GPIO4/5, as listed in
the [Seeed pinout](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).
GPIO5/6 would instead be D4/D5.

* OUT+ pulse width: **200 ns** (`PULSE_TICKS = 16` at 80 MHz), unchanged.
* OUT- HIGH width: **200–987.5 ns**, preserving the original complementary
  phase widths for the 128-value sweep.
* Dead time at **each** handoff: **10% of the configured OUT+ pulse width**,
  rounded up to the next 12.5 ns RMT tick. At 200 ns, the requested 20 ns
  becomes **25 ns** (`DEAD_TICKS = 2`). Both outputs are LOW during this gap.
* OUT+ rising-edge interval:
  `400 ns + (value >> 1) * 12.5 ns + 2 * dead_time`, from **450 ns to
  1237.5 ns** with the defaults. Adding gaps without shortening either phase
  extends the original interval by 50 ns; it no longer fits the old
  400–1200 ns window.
* The 128-value sweep is pre-rendered. OUT+ uses the S3's single DMA TX
  channel; OUT- uses three hardware memory blocks (144 symbols), enough for
  its 129-symbol frame plus EOF. No mid-frame refill interrupt is needed.
* The RMT sync manager starts both channels together and is reset after each
  completed burst. Both outputs remain LOW during the 100 us inter-burst gap.
* D0 runs independently of PPM. `PWM_DUTY_PERCENT = 100` holds it HIGH;
  `0` holds it LOW. For `1..99`, LEDC generates continuous PWM at
  `PWM_FREQ_HZ` (currently 100 kHz), including between PPM bursts.
  PWM phase is not synchronized to PPM.

For each payload value, the sequence is:

```text
OUT+ HIGH for PULSE_TICKS
both LOW for DEAD_TICKS
OUT- HIGH for BASE_TICKS + (value >> 1) - PULSE_TICKS
both LOW for DEAD_TICKS
```

Use a scope to check D0's frequency/duty and the complementary PPM waveform
after flashing. Compilation alone does not verify physical output timing.

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
automatically changes the gap. The frame size is limited to 128 values so
the complete OUT- waveform fits in hardware RAM.
PWM frequency and duty are separate parameters. For example, set
`PWM_FREQ_HZ = 100000` and `PWM_DUTY_PERCENT = 50` for 100 kHz at 50% duty.
At 0% or 100%, GPIO drives a constant level and frequency is unused.
Intermediate duties are rounded to the timer's available resolution, chosen
automatically from the board's 40 MHz crystal and requested frequency.
