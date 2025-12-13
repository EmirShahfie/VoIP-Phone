# VoIP-Phone

ESP32-S3 + ZephyrOS prototype of a VoIP-capable handset that exercises the WM8960 audio codec. The current firmware boots, brings up the codec over I2C, and lets you toggle between headphone and speaker output with a user button.

## What it does
- Configures the WM8960 codec for 44.1 kHz, 16-bit playback over I2S clocks while powering the required analog paths.
- Exposes a simple audio mode state machine (`AUDIO_MODE_HEADPHONE` vs `AUDIO_MODE_SPEAKER`) managed in `wm8960.c`.
- On boot: initializes the codec, defaults to headphones, and listens for a button interrupt to switch outputs.

## Hardware
- Board: ESP32-S3 DevKitC (Zephyr target) with Zephyr overlays mapping `audio0` (WM8960) and `user_button` GPIO.
- Audio: WM8960 codec wired on I2C (control) and I2S (audio data). Headphone and speaker outputs are both available. The WM8960 Audio HAT is used for this prototype, with customised pinouts needed.
- Input: Single GPIO button for toggling output path.

## Firmware flow (high level)
1) Verify I2C bus readiness and set up the button interrupt handler.  
2) Initialize the WM8960: reset, enable references/VMID, configure PLL and clocks, enable DACs, and set 0 dB headphone gain.  
3) Default to headphone mode; button press schedules work to flip between headphone and speaker paths via safe I2C writes from a Zephyr work item.

Key entry points: main control loop in [src/main.c](src/main.c), codec driver routines in [src/wm8960.c](src/wm8960.c) with register definitions in [include/wm8960.h](include/wm8960.h).

## Building and flashing
- Ensure the ESP32 Zephyr toolchain is installed and `west` is available.
- In Makefile, ensure that the Zephyr venv environment is adapted to your own.
- From the project root: `west build -b esp32s3_devkitc`
- Flash: `west flash`. Monitor: `west espressif monitor` or `west debug` per your setup.

## Next steps
- Add the SIP/VoIP stack and audio pipeline (I2S RX/TX) to carry real calls.
- Expose volume and mute controls; surface status over a UART/USB console.
- Add low-power behaviors and call-state UI (LEDs or display).

