# XIAO ESP32-S3 PPM transmitter

Port of `../ppm_tx` for the **Seeed Studio XIAO ESP32-S3**, with an independent
continuous PWM output. PPM keeps the original pre-rendered RMT DMA waveform.

## Outputs

| Signal | XIAO pin | ESP32-S3 GPIO | Behavior |
|--------|----------|---------------|----------|
| PWM | D0 | GPIO1 | 1 MHz, 50% duty (500 ns high / 500 ns low), LEDC |
| PPM OUT+ | D4 | GPIO5 | RMT TX, idle low |
| PPM OUT- | D3 | GPIO4 | Inverted mirror of the same RMT signal, idle high |

The board labels determine the pin mapping: D3/D4 are GPIO4/5, as listed in
the [Seeed pinout](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).
GPIO5/6 would instead be D4/D5.

* PPM pulse width: **200 ns** (16 ticks at 80 MHz).
* Rising-edge interval: `400 ns + (value >> 1) * 12.5 ns`, from **400 ns to
  1187.5 ns** for values 0..127.
* Each burst contains the 128-value sweep, sent as one DMA transfer, followed
  by a 100 us gap.
* OUT- uses the GPIO matrix to invert OUT+'s internal signal, so both legs
  share one RMT channel. PWM runs independently and continues between bursts;
  its phase is not synchronized to PPM.

Use a scope to check D0's frequency/duty and the complementary PPM waveform
after flashing. Compilation alone does not verify physical output timing.

At startup, `PWM pad check` reports a hardware pulse-counter readback from
GPIO1 after all output peripherals are configured. It counts both edges for
about 1 ms; roughly 2000 edges indicates a 1 MHz waveform. The reported
frequency is approximate because starting/stopping the measurement adds
software overhead. An error is logged if the result differs by more than 5%.
This checks transitions at the chip's pad, but does not measure duty cycle,
voltage amplitude, or the connection between the pad and your probe.

If D0 appears flat, probe D0 relative to a board GND with a high-impedance
input (for example, a scope's 1 MOhm input and a 10x probe), and use a timebase
around 200 ns/div. Check the pin label and header/solder contact. A meter
does not display the 1 MHz waveform.

## Build on Windows

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

`main/main.c` defines `PWM_GPIO`, `PWM_FREQ_HZ`, `OUT_A_GPIO`, `OUT_B_GPIO`,
`PULSE_TICKS`, `BASE_TICKS`, `N_VALUES`, and `INTER_BURST_US`.
The one-bit LEDC timer and duty of 1 give a fixed 50% PWM duty cycle.
