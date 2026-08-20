# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

TxOneMove: a custom hand-held RC transmitter (single-hand throttle + BNO055 orientation control) built around a Seeed XIAO ESP32-S3. The repo has four largely independent parts:

- `Firmware/` — PlatformIO/Arduino C++ firmware running on the ESP32-S3. This is where almost all engineering work happens.
- `WebApp/` — an installable PWA (`index.html` + `manifest.json` + `service-worker.js` + `icons/`) that connects to the transmitter over Web Bluetooth to show live telemetry and edit tunable parameters.
- `Python/` — desktop helper scripts (serial log visualization, LeCroy UART capture) used for debugging/tuning off-device.
- `Case/` — 3D-printable enclosure (STL/3MF), not source code.

German comments/log strings are common throughout the firmware (this is a German-speaking hobby project) — keep new comments consistent with the surrounding language when editing existing files.

## Firmware: build, flash, monitor

The firmware is a PlatformIO project rooted at `Firmware/` (open `Firmware/Firmware.code-workspace` in VS Code with the PlatformIO extension). Target env is `seeed_xiao_esp32s3`.

```
cd Firmware
pio run                 # build
pio run -t upload       # build + flash
pio device monitor      # serial monitor (115200 baud)
pio run -t upload -t monitor  # flash then immediately monitor
```

There is no test suite — validate changes by building, flashing to hardware, and observing serial output / BLE telemetry (the debug `Serial.println` calls in `setup()` and `SensorToDigital`/`BluetoothComm` are the primary diagnostics; several are commented out to save cycles — re-enable locally when debugging, don't leave them enabled in commits unless intentional).

`radioData.resetData()` in `main.cpp::setup()` (commented out) does a factory reset of all `Preferences` (NVS) storage — only uncomment temporarily when you actually want to wipe stored trims/models.

## Firmware architecture: the pipeline

Every stage of the control pipeline is a small class deriving from `RadioClass` (`RadioClass.hpp`), constructed once in `main.cpp` and driven every 20 ms tick from `loop()`, always in this fixed order:

```
AnalogToDigital → SensorToDigital → DigitalToFunction → Expo → Trim → DualRate → Mixer → FunctionToChannel → (tick boundary) → Transmitter → BluetoothComm
```

Each stage's `doFunction()` reads from and writes to a single shared `RadioData` instance (`RadioData.hpp`) — there is no other inter-module communication. `RadioData` is a plain aggregate of nested structs (`RawData`, `AnalogData`, `DigitalData`, `FunctionData`, `ChannelData`, plus the `*Data` config structs for each stage) that acts as the pipeline's blackboard. When adding a new stage or field, follow this pattern: add the struct field(s) to `RadioData`, read/write them from your `RadioClass` subclass, and wire the class into `main.cpp` in the correct pipeline position.

Rough responsibility of each stage:
- **AnalogToDigital** — reads ADC (throttle, battery) and the arm button/GPIO, converts to normalized digital values, handles button debounce/long-press and battery warning.
- **SensorToDigital** — reads the BNO055 IMU (I2C) and GPS (UART2/TinyGPS++), produces pitch/roll/orientation/GPS digital data, drives vibration feedback.
- **DigitalToFunction** — arm/disarm state machine, trim capture on long-press, LED/vibration output, zeroes controls when disarmed.
- **Expo / Trim / DualRate** — per-axis shaping of pitch/roll/throttle (expo curve, trim offset, rate scaling).
- **Mixer** — mixes throttle into pitch and produces V-tail left/right outputs from pitch+roll.
- **FunctionToChannel** — maps abstract `Function`s (PITCH/ROLL/VTAIL_LEFT/VTAIL_RIGHT/THROTTLE/NONE) onto up to `SUPPORTED_CHANNELS` (8) RC channels per the stored per-model mapping, applies inversion and channel limits, writes final `channelData.channel[]` (11-bit CRSF range 172–1811, center 992).
- **Transmitter** — packs `channelData` into a CRSF `RC_CHANNELS_PACKED` frame and writes it over `Serial1` to the TX module (half-duplex on one GPIO, direction flipped around each send); also parses incoming CRSF telemetry frames (battery voltage, barometric altitude, vertical speed) from the RX side of the same UART.
- **BluetoothComm** — BLE GATT server (`BLEDevice`) exposing one NOTIFY characteristic (periodic telemetry push, ~10 Hz) and one WRITE characteristic (command/parameter updates from the app). This is the only bridge to `WebApp/`.

Persistence (`RadioData::store*/load*`) uses ESP32 `Preferences` (NVS) under namespace `"Global"` for shared settings and `"Model-N"` (N = 0..`MAX_NUMBER_OF_MODELS`-1) for per-model settings (expo/dual-rate/trim/mixer/channel mapping/model name). Preference keys are short/abbreviated (e.g. `"atdd.tl.min"`) to fit NVS key-length limits — keep that convention when adding new persisted fields, and add both a `store` and matching `load` (with a sensible default) together.

## The BLE wire protocol (firmware ↔ WebApp)

`Firmware/src/BluetoothComm.hpp` and `WebApp/index.html` independently implement the *same* binary layout — there is no shared schema file, so **when changing one side you must update the other by hand**:

- Service/characteristic UUIDs, `CMD_*` command IDs, and `PARAM_*` parameter IDs are duplicated as `#define`s in `BluetoothComm.hpp` and as JS `const`s in `index.html` (search for `PARAM_` / `CMD_` in both files).
- The NOTIFY payload is a hand-packed little-endian byte buffer built field-by-field in `BluetoothComm::doFunction()` (grouped in comments as TX_DATA/RX_DATA/GPS_DATA/PARAM_DATA); `index.html`'s BLE notification handler (`readFloat32`/`readFloat64`/`readUint16`/`readUint8` around line 870+) decodes it with the exact same field order and widths. Field order, byte width, and units (e.g. many percentages are transmitted ×100) must match exactly on both ends.
- The WRITE characteristic carries a `CMD_*` byte, and for `CMD_UPDATE_PARAM` a `PARAM_*` id byte plus a little-endian float — decoded in `BluetoothComm.hpp`'s `RxCallback::onWrite`.

## WebApp: PWA install & offline cache

`WebApp/service-worker.js` precaches the app shell (`index.html`, `manifest.json`, `icons/*`, and the Leaflet + leaflet-control-compass CDN files) into a versioned cache (`CACHE_VERSION` at the top of the file) so the app installs on a phone and works fully offline, including the map UI chrome. OpenStreetMap tiles are cached opportunistically at runtime (cache-first, capped at `MAX_TILE_CACHE_ENTRIES`) so previously viewed map areas stay available offline, but the whole map isn't pre-downloaded.

**Whenever you change any precached file** (`index.html`, `manifest.json`, or anything under `icons/`), bump `CACHE_VERSION` in `service-worker.js` — that's what makes the browser notice the service worker changed and start the update flow; without it, installed clients keep serving the old cached files indefinitely. Behavior: while online the page calls `registration.update()` on load, on reconnect, and hourly; if a new worker installs, a toast ("Neue Version verfügbar") lets the user apply it on their own terms via `postMessage({type:'SKIP_WAITING'})` + reload, rather than silently swapping the app shell under a live BLE session.

The `icons/` PNGs were generated programmatically (no image tool in this environment) — see the gradient-ring mark matching the CSS `--accent`/`--accent-2` brand colors; regenerate with any image tool if the brand mark changes, keeping the same filenames/sizes referenced from `manifest.json` and the `<link rel="apple-touch-icon">`/`<link rel="icon">` tags in `index.html`.

## Python helper scripts

Standalone, run directly with `python <script>.py` (no build/package step, no requirements.txt in-repo — check imports at the top of each script for needed packages: `pyserial`, `numpy`, `matplotlib`, `contextily` for `visualization.py`). They read live serial output from the transmitter's USB-CDC port for logging/plotting (`visualization.py`, port/baud hardcoded near the top) and UART capture from a LeCroy scope (`LeCroy_Uart.py`) — adjust the hardcoded `PORT`/config constants at the top of each file for your setup rather than adding CLI args.
