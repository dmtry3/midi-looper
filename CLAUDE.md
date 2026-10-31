# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP32-S3 MIDI looper firmware: a single Arduino sketch, `midi-looper.ino` (~2000 lines). It records Note On/Off and CC events from DIN MIDI and USB MIDI, loops them against an external MIDI clock, and plays them back to DIN and USB. You control it with a pedal, 4 buttons, a rotary encoder, an SSD1315 OLED, and a BLE text-command interface. `README.md` (in Russian) is the user manual: pinout, controls, menu, BLE commands. Keep it in sync when behavior changes. Code comments are also in Russian.

There are no tests, linter, or build scripts. The only check is compiling the sketch and testing it on hardware.

## Build

The sketch needs the arduino-esp32 core **3.x** (it uses the 3.x `ledcAttach` API). It also needs the libraries **MIDI Library** (FortySevenEffects), **Adafruit GFX** and **Adafruit SSD1306**. BLE, LittleFS and USBMIDI come with the core.

`USB_MIDI_ENABLED 1` (the default) gives a compile `#error` on any chip other than S2/S3. It also needs these board settings: USB Mode = USB-OTG (TinyUSB) and USB CDC On Boot = Enabled. With arduino-cli (`~/.local/bin/arduino-cli`). Verified to compile with core esp32:esp32@3.3.12, MIDI Library 5.0.2, Adafruit GFX 1.13.0, Adafruit SSD1306 2.5.17 (≈60% flash on the default partition). The expected warnings are `volatile ++` deprecation (`advanceTick`) and the `BLE2902` deprecation (the core's BLE is NimBLE-based):

```sh
arduino-cli core install esp32:esp32
arduino-cli lib install "MIDI Library" "Adafruit GFX Library" "Adafruit SSD1306"
arduino-cli compile -b esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc .
arduino-cli upload  -b esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc -p <port> .
```

The compile-time feature flags (`USB_MIDI_ENABLED`, `PEDAL_ENABLED`, `OLED_ENABLED`, `ECHO_GUARD_MS`, `DEFAULT_CH_MASK`, `VIZ_*`) are `#define`s at the top of the sketch.

## Architecture

**Time is measured in MIDI clock ticks, not milliseconds.** `PPQN = 24` and `BAR_TICKS = 96`. `currentTick` advances on every incoming F8 (`handleClock` → `advanceTick`). If no external clock has arrived for 1.2 s, `handleInternalClock` generates ticks at the last smoothed tempo (default ≈110 BPM). Event timestamps, `loopLenTicks`, `recStartTick` and `playStartTick` are all in ticks, so the loop follows tempo changes. The playback position is `(currentTick - playStartTick) % loopLenTicks`. Timestamps are `uint16_t`, which is why the loop length is capped at 65000 ticks (677 bars, about 22 min at 120 BPM).

- BPM is a moving average over a 48-tick ring buffer of `micros()` (`updateTempo`). `loopBeatDuration` (ms per beat) is derived from it and is used only for display and LED timing.
- `bleBeatMs` is a *frozen* ms-per-beat, captured at record start or preset load. Its only job is converting ticks to ms for the BLE client (`ticksToBleMs`), which expects ms. Don't use it in the engine.
- When a clock arrives on both DIN and USB, `acceptClockSource` latches the first active source.
- MIDI Start resets the position (`handleStart`) but not `looperState`. Continue does not reset the position.

**Quantized actions.** The pedal, BLE `/rec` and `/stop`, and preset loads only set flags (`pendingStart`, `pendingStop`, `pendingLoadSlot`). `triggerQuantizedAction()` runs on beat 1 of each bar, from `advanceTick`, and carries them out. A preset load requested while the loop is playing is handled in `playback()` instead, when the position wraps. A pending load takes priority over pending rec/stop.

**State machine.** `LooperState` is IDLE → RECORDING → PLAYING ⇄ OVERDUB. Each overdub increments `currentLayer`, and every `MidiEvent` stores its `layer`, which is how undo removes the last layer. `loop()` detects the transition out of OVERDUB and calls `onOverdubExit()`.

**Event store.** There is one flat array, `MidiEvent events[5000]` / `eventCount`. It is not kept sorted. `playback()` scans every event each loop iteration, using the per-event `played` flag, and clears all the flags when the position wraps. `recordEvent()` does most of the logic: the channel mask filter, echo guard (`lastSentNoteTime`), CC takeover (`ccTakeover`) and overdub CC overwrite (`ccOd`, `eraseCCRange`), on-the-fly quantization, and orphan note-off handling. Notes already held when recording starts are captured at tick 0 (`captureHeldNotesAtStart`).

**Concurrency.** `loop()` runs on core 1 and handles MIDI read, USB MIDI, inputs, playback, LED and BLE notifications. The OLED is drawn by `oledTask`, pinned to core 0 at 60 ms intervals. That task reads engine globals without locks; shared values are `volatile`, and helpers like `ticksSince()` are written to tolerate torn or out-of-order reads. Keep this in mind when adding state the display reads. BLE `onWrite` callbacks run on the BLE stack's task and call engine functions directly. The encoder is read in an ISR (`encoderISR`, quadrature table) that writes a delta, which `handleEncoder` consumes.

**UI.** `UiMode` (normal, menu list or edit, channels grid, save/load slot picker) is driven by the `handleMenuButtons`, `handleEncoder` and `handleEncoderSwitch` functions, and `handleUiTimeout` closes any mode after 12 s. The menu actions (`uiDo*`) wrap the same engine functions the BLE commands call (`halfLoop`, `doubleLoop`, `transposeLoop`, `undoLastLayer`, `applyQuantizeToLoop`, `savePreset`, `requestLoadPreset`). New features should be reachable from both the menu and BLE.

**Persistence (LittleFS).** Presets are stored as `/presetN.bin` (N = 1..5). Each file holds `PRESET_MAGIC`, `loopLenTicks` and `eventCount`, followed by a raw `memcpy` of the `MidiEvent` array. **Changing the `MidiEvent` struct or the tick semantics breaks existing presets.** If you do, bump `PRESET_MAGIC` so old files are rejected instead of misread. The `/chmask.bin` file stores the recording channel mask. Quantize settings are not persisted.

**BLE protocol.** Text commands go to a single characteristic (`MyCharacteristicCallbacks::onWrite`). Replies are notifications sent with `sendBleNotify`, in the same `/name args` format. After a preset load, `loop()` streams the whole loop to the client as `/rec_noteon`, `/rec_noteoff`, `/rec_cc` and `/rec_pos` messages (the "visual dump"), throttled by `VISUAL_DUMP_*_INTERVAL`. Position is sent as a normalized `/pos` every `BLE_INTERVAL` ms. The command list is in README §11.
