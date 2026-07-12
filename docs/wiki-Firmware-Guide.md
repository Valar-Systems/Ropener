# ⚙️ Firmware Guide

*For advanced users — what the Ropener firmware actually is, how it works, and how to flash or update it from the YAML file.*

Ropener runs on **[ESPHome](https://esphome.io)**, an open-source firmware framework. The entire behaviour of the device — the web page, the motor logic, the buttons, the schedule — is described in a single human-readable YAML file and compiled into a binary that runs on the on-board ESP32. There is no closed-source blob and no cloud account: if you can read the YAML, you can see (and change) everything the device does.

> 💡 If you only want to **use** the device, you don't need any of this — see the [User Guide](User-Guide). This page is for people who want to recompile, update, or modify the firmware themselves.

---

## Contents

1. [What is ESPHome?](#1-what-is-esphome)
2. [How the Ropener firmware is built](#2-how-the-ropener-firmware-is-built)
3. [Which YAML file is mine?](#3-which-yaml-file-is-mine)
4. [What you may and may not change](#4-what-you-may-and-may-not-change)
5. [Tools for flashing](#5-tools-for-flashing)
6. [Updating the firmware from the YAML](#6-updating-the-firmware-from-the-yaml)
7. [First flash vs. over-the-air (OTA) updates](#7-first-flash-vs-over-the-air-ota-updates)
8. [secrets.yaml and API encryption (optional)](#8-secretsyaml-and-api-encryption-optional)
9. [External components](#9-external-components)
10. [After flashing](#10-after-flashing)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. What is ESPHome?

ESPHome turns a **YAML description** of a device into real firmware. You write *what* the device should expose — a cover, some buttons, a few numbers, a web server — and ESPHome generates the C++, compiles it for the ESP32, and flashes it.

A few ideas worth knowing before you touch the file:

- **The YAML is the source of truth.** Nothing is configured by clicking around after the fact at the firmware level — every pin, entity, and behaviour comes from the file. Re-flashing the same YAML reproduces the same firmware exactly.
- **Entities, not screens.** ESPHome thinks in *entities*: a `cover`, a `switch`, a `number`, a `sensor`, etc. The built-in web page and Home Assistant both render whatever entities the YAML declares — that's why the web UI and Home Assistant always show the same controls.
- **It runs locally.** The device serves its own control page (`web_server:`) and talks to Home Assistant directly over the local network (`api:`). No internet round-trip is needed to move the curtain; the only thing that needs the network is clock sync for the schedule.
- **Two kinds of settings.** *Compile-time* values (like the gear ratio) are baked into the binary and need a re-flash to change. *Runtime* values (speed, travel distance, timezone…) are exposed as entities you edit in the browser, and they persist in flash across reboots and updates. See [Section 4](#4-what-you-may-and-may-not-change).

If you've never used ESPHome, the official [Getting Started guide](https://esphome.io/guides/getting_started_command_line.html) is a good companion to this page.

---

## 2. How the Ropener firmware is built

At a high level, the curtain hardware is an **ESP32** microcontroller driving a **TMC2209** silent stepper driver, which turns a **NEMA-17 motor** and an **MK7 gear** that pulls the poly rope. The YAML wires all of that together. The main building blocks in the file:

| YAML section | What it does |
| --- | --- |
| `esphome:` / `esp32:` | Device name, firmware version, board variant, and the `on_boot:` sequence that restores saved settings into the motor driver. |
| `external_components:` | Pulls in the TMC2209 stepper component (see [Section 9](#9-external-components)). |
| `wifi:` / `captive_portal:` / `improv_serial:` | Wi-Fi provisioning. The firmware ships with **no** saved network, so a fresh unit broadcasts a `ropener-XXXXXX` setup hotspot. |
| `web_server:` | The built-in control page at `http://ropener-XXXXXX.local`, grouped into Control / Setup / Schedule / Motion / StallGuard / Diagnostics. |
| `api:` | The native ESPHome API used by Home Assistant (optional, unencrypted by default). |
| `ota:` | Over-the-air update support, so later updates don't need a USB cable. |
| `stepper:` + `cover:` | The motion engine. Position is tracked in motor *steps* and reported to the UI as a 0–1 (closed→open) ratio. |
| `binary_sensor:` (buttons) | The three physical buttons (close/home, open, Wi-Fi reset). |
| `number:` / `select:` / `switch:` / `text:` / `datetime:` | The runtime-tunable settings — travel distance, speed, motor direction, schedule times, timezone, latitude/longitude, StallGuard thresholds. |
| `time:` + `sun:` + `interval:` | The daily schedule, including the optional sunrise/sunset mode. |

You don't need to understand every line to update the firmware — but it's all commented in the file itself if you want to dig in.

---

## 3. Which YAML file is mine?

The YAML source for each board lives in its own sub-folder under [`firmware/`](https://github.com/Valar-Systems/Ropener/tree/main/firmware); the matching **pre-built binaries are attached to each [release](https://github.com/Valar-Systems/Ropener/releases/latest)**. Every release provides two binaries per board — a `*.factory.bin` (full image for the first USB flash) and a `*.ota.bin` (the over-the-air update image you upload from the device's web page). So you can build from source, flash the factory image with no tools, or update an already-running device straight from its browser (see [Section 6](#6-updating-the-firmware-from-the-yaml)). The v2.6.1 files:

| Board | Microcontroller | YAML source | Factory image — first USB flash | OTA image — Wi-Fi update |
| --- | --- | --- | --- | --- |
| **VAL3100** | ESP32-**C6** | [`Ropener-VAL3100-2.6.1.yml`](https://github.com/Valar-Systems/Ropener/blob/main/firmware/VAL3100/Ropener-VAL3100-2.6.1.yml) | [`Ropener-VAL3100-2.6.1.factory.bin`](https://github.com/Valar-Systems/Ropener/releases/download/v2.6.1/Ropener-VAL3100-2.6.1.factory.bin) | [`Ropener-VAL3100-2.6.1.ota.bin`](https://github.com/Valar-Systems/Ropener/releases/download/v2.6.1/Ropener-VAL3100-2.6.1.ota.bin) |
| **VAL3000** | ESP32-**C3** | [`Ropener-VAL3000-2.6.1.yml`](https://github.com/Valar-Systems/Ropener/blob/main/firmware/VAL3000/Ropener-VAL3000-2.6.1.yml) | [`Ropener-VAL3000-2.6.1.factory.bin`](https://github.com/Valar-Systems/Ropener/releases/download/v2.6.1/Ropener-VAL3000-2.6.1.factory.bin) | [`Ropener-VAL3000-2.6.1.ota.bin`](https://github.com/Valar-Systems/Ropener/releases/download/v2.6.1/Ropener-VAL3000-2.6.1.ota.bin) |

> ⚠️ **The board variants are not interchangeable.** The C3 and C6 use different GPIO pin assignments, so flashing the wrong board's YAML or binary will leave the buttons and motor mis-wired. If you're not sure which board you have, check the silk-screen label on the PCB.

> 🏠 **Want Alexa / Apple Home / Google Home?** You don't need a separate firmware for that — pair the standard build with [Matterbridge](https://github.com/Luligu/matterbridge) to bridge the curtain into those ecosystems. See the [README](https://github.com/Valar-Systems/Ropener#readme).

---

## 4. What you may and may not change

*This section only matters if you **build from the YAML** (Path B). If you flash the ready-made binary (Path A), there's nothing to edit — skip to [Section 6](#6-updating-the-firmware-from-the-yaml).*

When building from source, the one thing most people change is the device name in the `esphome:` block, to give the unit a friendly label:

```yaml
esphome:
  name: ropener-bedroom-1                 # optional — a friendly name
  friendly_name: "Ropener Bedroom 1"      # optional
```

You don't have to rename it: `name_add_mac_suffix: true` already makes every unit unique as `ropener-XXXXXX`. Change it only if you want something more memorable.

Everything else should be left alone unless you know exactly what you're doing. In particular:

- **Don't touch the `substitutions:` block** (`gear_distance_cm`, `steps_per_revolution`). These are mechanical constants tied to the actual gear and microstepping; changing them breaks the distance calibration.
- **Don't touch the pin numbers** in `uart:`, `stepper:`, and the button `binary_sensor:` entries — they match the PCB.
- **You don't need to edit travel distance, speed, schedule, timezone, etc. in the YAML.** Those are runtime settings: flash once, then set them from the web page (they persist across reboots and future updates). See the [ESPHome Guide](ESPHome-guide) and [User Guide](User-Guide).

> 🛑 **StallGuard warning.** Do **not** enable or tune StallGuard-based auto-homing unless you have set it up correctly — it's finicky and needs exact `SGTHRS`/`TCOOLTHRS` values. It is not required for the device to work; home the curtain manually instead (see the [User Guide](User-Guide#5-homing-calibrating-the-closed-position)).

---

## 5. Tools for flashing

Only ESPHome itself can **compile** the YAML into firmware — a web page can't. Pick whichever tool fits how you work:

| Tool | What it does | Notes |
| --- | --- | --- |
| **ESPHome CLI** (`pip install esphome`) | Compiles the YAML **and** flashes it | `esphome run file.yml` — over USB or OTA. Needs [Python](https://www.python.org/). |
| **ESPHome Dashboard** | Compiles **and** flashes, via a local web UI | Standalone (`esphome dashboard`) or the Home Assistant **ESPHome** add-on. USB and OTA. |
| **ESPHome Web** ([web.esphome.io](https://web.esphome.io)) | **Flashes a pre-built `.bin`** — does *not* compile | Runs in Chrome/Edge, no install, USB only (no OTA). Flash the ready-made `.factory.bin` we ship ([Section 3](#3-which-yaml-file-is-mine)) — or one you compiled yourself. |

In short: the CLI and the Dashboard turn the YAML into firmware; ESPHome Web just writes a finished binary — and since we ship one in each board's folder, a standard install needs no compiling at all.

---

## 6. Updating the firmware from the YAML

How you get firmware onto the device depends on its state — pick the matching method:

- **Already running Ropener?** Update it straight from its web page — no tools, no USB (see just below).
- **New / blank board?** Flash the factory image over USB — **Path A**.
- **Want to customize the YAML or script updates?** Build from source — **Path B**.

> 📦 **Two binary types.** First-time USB flashing uses the **factory image** (`*.factory.bin`). Over-the-air updates use the **OTA image** (`*.ota.bin`, ESPHome's "Modern format"). Each board folder ships both ([Section 3](#3-which-yaml-file-is-mine)) — don't upload a factory image to an OTA updater.

### Update in the device's web page (already running ESPHome — no tools)

If the device already runs Ropener firmware and is on your network, this is the simplest way to update it — no USB cable and nothing to install:

1. Open the device's control page (`http://ropener-XXXXXX.local`) in any browser on the same network.
2. Scroll to the **OTA Update** card.
3. Click **Choose File** and select the new **OTA image** — your board's `*.ota.bin` (e.g. `Ropener-VAL3100-2.6.1.ota.bin`), **not** the `*.factory.bin`.
4. Click **Update**. The device flashes the new build and reboots; your Wi-Fi and saved settings are kept.

> 💡 You can push the same OTA from the ESPHome CLI/Dashboard instead (Path B, over the network) — handy for scripting or bulk updates.

### Path A — Flash the ready-made binary with ESPHome Web (no install)

Each board folder ships a pre-compiled `*.factory.bin`, so you can flash without installing ESPHome or compiling anything.

1. Download your board's `*.factory.bin` from [Section 3](#3-which-yaml-file-is-mine) (e.g. `Ropener-VAL3100-2.6.1.factory.bin`).
2. Plug the device into your computer over USB.
3. Go to **[web.esphome.io](https://web.esphome.io)** in Chrome or Edge and click **Connect**; pick the device's serial port.
4. Click **Install**, choose the `.factory.bin` you downloaded, and let it flash.

> 💡 **Nothing to configure first.** The stock build names itself `ropener` plus a per-device MAC suffix, so every unit comes up unique as `ropener-XXXXXX`. Want a custom friendly name, API encryption, or different motion defaults? Build from source instead (Path B).

### Path B — Build from the YAML with the ESPHome CLI or Dashboard

Use this to customize the firmware, or to update over Wi-Fi (OTA) after the first flash.

1. Install ESPHome: `pip install esphome` (or use the Home Assistant **ESPHome** add-on / `esphome dashboard`).
2. Save the correct YAML for your board ([Section 3](#3-which-yaml-file-is-mine)) locally and edit the device name ([Section 4](#4-what-you-may-and-may-not-change)).
3. Compile and flash in one step:
   ```bash
   esphome run Ropener-VAL3100-2.6.1.yml
   ```
   The first time, choose the **USB/serial** port. Once the device is on your network the same command offers an **OTA (Over-The-Air)** option — no cable needed.
4. Watch the boot log to confirm it came up cleanly:
   ```bash
   esphome logs Ropener-VAL3100-2.6.1.yml
   ```

> 🔧 **Flashing a customized build through the browser?** Run `esphome compile Ropener-VAL3100-2.6.1.yml` to produce your own `*.factory.bin` (ESPHome writes it under `.esphome/build/<device-name>/…`), then flash it with Path A from step 2.

> 📝 **Wi-Fi survives updates.** Re-flashing does **not** erase your saved Wi-Fi network or your runtime settings — they live in a separate area of flash. You only re-provision Wi-Fi after a deliberate Wi-Fi reset (hold Button 3, see the [User Guide](User-Guide#9-resetting-wi-fi)).

---

## 7. First flash vs. over-the-air (OTA) updates

- **First flash must be over USB.** A brand-new or blank chip has no firmware to receive an OTA, so the very first install needs a cable.
- **Every update after that can be OTA** — either upload the `*.ota.bin` in the device's **OTA Update** web card ([Section 6](#6-updating-the-firmware-from-the-yaml)), or push it from the ESPHome CLI/Dashboard (pick the device's network address instead of a serial port). Both use the `*.ota.bin` image, not the factory image.
- **OTA requires the device to be on the same network** and reachable at `ropener-XXXXXX.local` (or its IP address).

---

## 8. secrets.yaml and API encryption (optional)

The firmware compiles **without a `secrets.yaml`** out of the box — there are no hardcoded Wi-Fi credentials or keys to supply.

The only reason to create one is to enable **encryption on the Home Assistant API**. By default the native API is unencrypted, and Home Assistant shows a "Communication not encrypted" warning when you first add the device. The device still works normally. To silence the warning:

1. Generate a 32-byte base64 key:
   ```bash
   python3 -c "import secrets,base64; print(base64.b64encode(secrets.token_bytes(32)).decode())"
   ```
2. Put it in a `secrets.yaml` next to the firmware:
   ```yaml
   api_key: "your-generated-key-here"
   ```
3. Uncomment the encryption block in the `api:` section of the YAML:
   ```yaml
   api:
     encryption:
       key: !secret api_key
   ```
4. Re-flash, then add the key in Home Assistant when prompted.

---

## 9. External components

The TMC2209 stepper driver isn't part of core ESPHome, so the firmware pulls it from a community repository at compile time:

```yaml
external_components:
  - source: github://slimcdk/esphome-custom-components
    components: [tmc2209_hub, tmc2209, stepper]
```

ESPHome downloads and caches this automatically on the first build — you don't need to install anything by hand. It does mean the **first compile needs an internet connection**; later builds use the cached copy.

---

## 10. After flashing

A successful flash gets you a running device, but it still needs to be set up:

1. **Provision Wi-Fi** — connect to the `ropener-XXXXXX` hotspot (or use Improv over USB). See [User Guide §2](User-Guide#2-first-time-setup-connect-to-wi-fi).
2. **Configure the device** — set travel distance (Centimeters), motor direction, and home the curtain. See the [ESPHome Guide](ESPHome-guide).
3. **Use it** — schedule, positions, Home Assistant, etc. See the [User Guide](User-Guide).

---

## 11. Troubleshooting

| Symptom | What to try |
| --- | --- |
| Compile fails on first build | Check your internet connection — ESPHome needs to download the external TMC2209 component once. |
| "Wrong" buttons or motor won't move | You may have flashed the wrong board's YAML. Confirm VAL3000 (C3) vs. VAL3100 (C6) and re-flash the matching file ([Section 3](#3-which-yaml-file-is-mine)). |
| Can't flash over USB | Try a different (data-capable) USB cable, and a Chromium-based browser for ESPHome Web. Some boards need to be put into bootloader mode. |
| OTA option doesn't appear | The device must already be running this firmware and reachable on the network. Do the first flash over USB. |
| OTA update rejected / "invalid image" | You picked the wrong file — the **OTA Update** card needs the `*.ota.bin` (Modern format), not the `*.factory.bin` (that's only for the initial USB flash). |
| Home Assistant says "not encrypted" | Expected — the device works anyway. Enable API encryption if you want to remove it ([Section 8](#8-secretsyaml-and-api-encryption-optional)). |
| Lost Wi-Fi / settings after update | Re-flashing does not erase them; if Wi-Fi is gone, the device was likely Wi-Fi-reset. Re-provision via the hotspot ([User Guide §2](User-Guide#2-first-time-setup-connect-to-wi-fi)). |

---

*Still stuck? [Open an issue](https://github.com/Valar-Systems/Ropener/issues) — and see the [ESPHome Guide](ESPHome-guide) and [User Guide](User-Guide) for day-to-day configuration.*
