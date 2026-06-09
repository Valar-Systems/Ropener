# Ropener Curtain Controller — User Guide

*Firmware 2.3.0*

---

Ropener is a Wi-Fi connected curtain/cover controller. It pulls a beaded rope with a small stepper motor to open and close your curtains. You can control it from any web browser on your network — no Home Assistant or cloud account required — or pair it with Home Assistant if you use one. It supports a daily open/close schedule, automatic position calibration, and several tuning options.

---

## Contents

1. [At a Glance](#1-at-a-glance)
2. [First-Time Setup: Connect to Wi-Fi](#2-first-time-setup-connect-to-wi-fi)
3. [Open the Web Control Panel](#3-open-the-web-control-panel)
4. [Everyday Use](#4-everyday-use)
5. [Homing (Calibrating the Closed Position)](#5-homing-calibrating-the-closed-position)
6. [Set Up the Daily Schedule](#6-set-up-the-daily-schedule)
7. [Setting Your Timezone](#7-setting-your-timezone)
8. [Timezone POSIX Strings](#8-timezone-posix-strings)
9. [Resetting Wi-Fi](#9-resetting-wi-fi)
10. [Adjusting How It Moves (Advanced)](#10-adjusting-how-it-moves-advanced)
11. [Home Assistant (Optional)](#11-home-assistant-optional)
12. [Troubleshooting](#12-troubleshooting)
13. [Quick Reference](#13-quick-reference)

---

## 1. At a Glance

- **Control from a browser:** open the device's web page on your phone or computer.
- **Three ways to operate:** the web page, the physical buttons on the device, or Home Assistant.
- **Open / close / partial:** drive the curtain fully open, fully closed, or to any position in between.
- **Daily schedule:** set a time to open and a time to close automatically every day.
- **Self-calibrating:** the device finds the fully-closed position by itself (“homing”).

---

## 2. First-Time Setup: Connect to Wi-Fi

The device ships without any saved Wi-Fi network, so the first thing to do is tell it which network to join. There are two ways to do this.

### Option A — The setup hotspot (recommended)

- Power on the device and wait about a minute.
- On your phone or laptop, open the Wi-Fi network list. You will see a network named **“ropener-XXXXXX”**, where XXXXXX is a unique code for your device.
- Write down or take a photo of this full name now — you will need the same “ropener-XXXXXX” to open the device in your browser later (see Section 3). The hotspot disappears once setup is done, so note it before continuing.
- Connect to it. A setup page should open automatically (this is the “captive portal”). If it does not, open a browser and go to `http://192.168.4.1`.
- Choose your home Wi-Fi network from the list, enter its password, and save.
- The device will reboot and join your network. The setup hotspot disappears once it connects successfully.

> **Remember:** the “ropener-XXXXXX” code is also your device's web address — you'll type `http://ropener-XXXXXX.local` into a browser to control it. Keep the code handy.

### Option B — Over USB (Improv)

If the device is plugged into a computer over USB, you can provision it from a web tool that supports “Improv” (for example, [web.esphome.io](https://web.esphome.io)). Open the tool, connect to the device's serial port, and enter your Wi-Fi details when prompted.

> **Note:** Your Wi-Fi details are stored on the device and kept across reboots and firmware updates. You only need to do this once (unless you reset Wi-Fi — see Section 9).

---

## 3. Open the Web Control Panel

Once the device is on your network, open a web browser on the same network and go to:

```
http://ropener-XXXXXX.local
```

Replace XXXXXX with your device's code (the same code shown in the setup hotspot name). The page lists every control and reading described in this guide. Bookmark it for easy access.

> **Tip:** If the `.local` address does not load, your network may not support it. In that case, find the device's IP address in your router and use `http://<that-ip>` instead.

---

## 4. Everyday Use

### From the web page or Home Assistant

- **Open:** raises the cover to the fully-open position.
- **Close:** lowers the cover to the fully-closed position.
- **Set a position:** drag the cover slider to any point from 0% (closed) to 100% (open).
- **Stop:** halts movement immediately and leaves the cover where it is.

### Using the physical buttons

The device has three buttons. While the cover is moving, a press of Button 1 or Button 2 simply stops it.

| Button | Action | What it does |
| --- | --- | --- |
| Button 1 | Short press (about 1 sec) | Closes the cover (or stops it if moving). |
| Button 1 | Long press (3–10 sec) | Starts homing / calibration (see Section 5). |
| Button 2 | Press and release | Opens the cover (or stops it if moving). |
| Button 3 | Long press (3–10 sec) | Clears saved Wi-Fi and reboots into setup (see Section 9). |

---

## 5. Homing (Calibrating the Closed Position)

> ⚠️ **Important — you must stop homing manually (automatic stop is disabled)**
>
> The sensor that normally detects the closed end (called “StallGuard”) is not working in this version of the firmware. During homing the motor will **NOT** stop by itself. You must watch the curtain and press Stop the moment it reaches the fully-closed position. If you do not stop it, the motor will keep running and strain against the stop. Always stay with the device while homing.

“Homing” teaches the device where the fully-closed position is. The motor drives the curtain toward the closed end; when it arrives, you stop it, and the device marks that spot as 0% (closed). Everything else — the open position, percentages, and the schedule — is measured from there.

Run homing when:

- You first install the device.
- The reported position no longer matches reality (for example after the rope slipped or was pulled by hand).

### How to home (stop it yourself)

1. Start homing: press and hold Button 1 for 3 to 10 seconds, or click the “Start-Stop Homing” button on the web page.
2. Watch the curtain drive toward the closed end.
3. The instant it reaches the fully-closed position, press Stop — press Button 1 once, or click “Start-Stop Homing” again. This marks that spot as 0% (closed) and ends homing.

The “State” reading on the web page shows **HOMING** while homing is running and returns to **IDLE** once you press Stop.

---

## 6. Set Up the Daily Schedule

The device can open and close the curtain automatically at set times every day. Four controls on the web page work together:

| Control | What to set |
| --- | --- |
| Open Time | The time of day to open the cover (e.g. 07:00). |
| Close Time | The time of day to close the cover (e.g. 21:00). |
| Schedule Enabled | Turn this ON to run the schedule, OFF to pause it. |
| Timezone | Your local timezone, so the times mean what you expect (see Section 7). |

**Steps:**

1. Set your Timezone first (Section 7) — the open/close times are in local time.
2. Set Open Time and Close Time.
3. Turn Schedule Enabled ON.

> **Notes:** The schedule needs an internet connection to keep accurate time. Scheduled moves are skipped while the device is homing. The schedule is off by default, and all four settings are remembered across reboots.

---

## 7. Setting Your Timezone

The device keeps time using the internet, which provides time in UTC. To make the schedule fire at the correct local time, enter your timezone in the “Timezone” box using a POSIX timezone string. Copy the value for your region from the table in Section 8 and paste it into the Timezone box, then save.

**Example:** for U.S. Eastern time with daylight saving, enter `EST5EDT,M3.2.0,M11.1.0`.

> **Important about the number sign:** In a POSIX timezone string the number is the hours WEST of UTC, so it looks “backwards” compared to the usual UTC offset. U.S. Eastern (UTC−5) is written as 5, Pacific (UTC−8) as 8, and so on. The table in Section 8 already has this right — just copy it exactly, including capitalization and punctuation.

The trailing parts (for example `,M3.2.0,M11.1.0`) are the daylight-saving rules. The U.S. switches on the 2nd Sunday of March and the 1st Sunday of November; the European Union switches on the last Sunday of March and the last Sunday of October. The tables in Section 8 already encode the correct rule for each region — just copy the value exactly. If your area does not observe daylight saving time, use the “without DST” value, which has no trailing rules.

---

## 8. Timezone POSIX Strings

Find your region below and copy the matching string into the Timezone box. Use the “With DST” column if your area changes its clocks twice a year; use “Without DST (standard all year)” if it does not.

### United States

| Zone | Where it's used | With DST | Without DST (standard all year) |
| --- | --- | --- | --- |
| Eastern | New York, Florida, Ohio, most of the East Coast | `EST5EDT,M3.2.0,M11.1.0` | `EST5` |
| Central | Texas, Illinois, Tennessee (western), most of the Midwest | `CST6CDT,M3.2.0,M11.1.0` | `CST6` |
| Mountain | Colorado, Utah, Montana, New Mexico | `MST7MDT,M3.2.0,M11.1.0` | `MST7` |
| Mountain (Arizona) | Most of Arizona — no daylight saving | *(not used)* | `MST7` |
| Pacific | California, Washington, Oregon, Nevada | `PST8PDT,M3.2.0,M11.1.0` | `PST8` |
| Alaska | Most of Alaska | `AKST9AKDT,M3.2.0,M11.1.0` | `AKST9` |
| Hawaii–Aleutian (Aleutian Is.) | Western Aleutian Islands, AK — observes DST | `HST10HDT,M3.2.0,M11.1.0` | `HST10` |
| Hawaii | State of Hawaii — no daylight saving | *(not used)* | `HST10` |
| Atlantic | Puerto Rico, U.S. Virgin Islands — no daylight saving | *(not used)* | `AST4` |
| Samoa | American Samoa — no daylight saving | *(not used)* | `SST11` |
| Chamorro | Guam, Northern Mariana Islands — no daylight saving | *(not used)* | `ChST-10` |
| Wake Island | Wake Island — no daylight saving | *(not used)* | `<+12>-12` |

*Zones marked “(not used)” under With DST do not observe daylight saving time, so only the standard-time string applies. Daylight-saving rules shown reflect current U.S. law (spring forward 2nd Sunday of March, fall back 1st Sunday of November).*

### Europe

Most of Europe observes EU daylight saving (forward on the last Sunday of March, back on the last Sunday of October). Countries that do not change their clocks are listed with a single standard-time value.

| Zone | Where it's used | With DST | Without DST (standard all year) |
| --- | --- | --- | --- |
| Western European | Portugal (mainland), Canary Islands | `WET0WEST,M3.5.0/1,M10.5.0` | `WET0` |
| UK / Ireland | United Kingdom, Ireland | `GMT0BST,M3.5.0/1,M10.5.0` | `GMT0` |
| Central European | France, Germany, Spain, Italy, Netherlands, Belgium, Poland, Sweden, Norway, Denmark, Switzerland, Austria, Czechia — most of central Europe | `CET-1CEST,M3.5.0,M10.5.0/3` | `CET-1` |
| Eastern European | Greece, Finland, Romania, Bulgaria, Estonia, Latvia, Lithuania, Ukraine | `EET-2EEST,M3.5.0/3,M10.5.0/4` | `EET-2` |
| Azores | Azores (Portugal) | `<-01>1<+00>,M3.5.0/0,M10.5.0/1` | `<-01>1` |
| Iceland | Iceland — no daylight saving | *(not used)* | `GMT0` |
| Kaliningrad | Kaliningrad, Russia — no daylight saving | *(not used)* | `EET-2` |
| Moscow | Moscow / western Russia — no daylight saving | *(not used)* | `MSK-3` |
| Belarus | Minsk, Belarus — no daylight saving | *(not used)* | `<+03>-3` |
| Turkey | Türkiye — no daylight saving | *(not used)* | `<+03>-3` |

*Zones marked “(not used)” under With DST do not change their clocks. POSIX values are taken directly from the IANA time zone database.*

---

## 9. Resetting Wi-Fi

If you move the device to a new network, or want to hand it to someone else, you can erase the saved Wi-Fi details:

- Press and hold Button 3 for 3 to 10 seconds.
- The device erases its saved network and reboots.
- After rebooting it broadcasts the “ropener-XXXXXX” setup hotspot again — follow Section 2 to connect it to a network.

> **Note:** This only clears Wi-Fi credentials. Your schedule, timezone, and tuning settings are kept.

---

## 10. Adjusting How It Moves (Advanced)

These settings are optional. The defaults work for a typical installation; change them only if needed. All values are remembered across reboots.

| Setting | What it controls | Notes |
| --- | --- | --- |
| Centimeters | Total travel distance of the curtain, in cm. | Set this to your curtain's open-to-closed travel. Default 30 cm. |
| Speed | How fast the motor runs (steps/second). | Higher is faster but louder. Effective max is 2000. |
| Acceleration | How quickly the motor speeds up / slows down. | Higher is snappier; too high can skip or be noisy. |
| Motor Direction | Which way the motor winds the rope. | Flip this if Open and Close are reversed for your install. |
| IRUN value | Motor running current / torque (1–31). | Raise for more pulling force; lower if the driver runs hot. Default 25. |
| SGTHRS value | Sensitivity of the stall detection used for homing (0–255). | See tuning note below. |
| TCOOLTHRS value | Speed threshold above which stall detection is active. | See tuning note below. |

### Tuning stall-based homing (optional)

> **Note:** automatic stall detection is not working in the current firmware, so the SGTHRS / TCOOLTHRS settings have no effect yet — home manually by pressing Stop at the closed position (see Section 5). The guidance below applies once automatic detection is restored.

Homing is meant to detect when the motor meets resistance at the closed end. Two readings on the web page help you tune it:

- **SG_RESULT Sensor:** a live load reading that drops as the motor works harder.
- **TSTEP Sensor:** a measure of motor speed (smaller means faster).

To tune: run the cover under normal load while watching SG_RESULT, note the value it settles at, then set SGTHRS to roughly half of that. If homing stops too early (false stalls), lower SGTHRS; if it does not detect the stop, raise it. Leave these at their defaults unless homing is unreliable.

---

## 11. Home Assistant (Optional)

If you use Home Assistant, the device is discovered automatically over the local network and appears as a cover plus all the controls and readings in this guide. You do not need Home Assistant to use any feature — the web page offers the same controls.

When first added, Home Assistant may warn that communication is not encrypted. The device still works normally. To remove the warning, an encryption key can be enabled in the firmware configuration (a setup step for the installer).

---

## 12. Troubleshooting

| Symptom | What to try |
| --- | --- |
| Can't find the web page | Make sure your phone/PC is on the same Wi-Fi as the device. Try the device's IP address instead of the `.local` name. Confirm it joined Wi-Fi (the setup hotspot should be gone). |
| No setup hotspot appears | Wait a full minute after power-on. If it already joined a network, hold Button 3 for 3–10 seconds to reset Wi-Fi and try again. |
| Open/Close are reversed | Toggle the “Motor Direction” switch. |
| Position is wrong / drifted | Run homing (Section 5) to recalibrate. |
| Schedule fires at the wrong time | Check the Timezone value (Section 7/8) and that the device has internet for time sync. |
| Schedule doesn't run | Confirm “Schedule Enabled” is ON and that Open/Close times are set. |
| Homing motor doesn't stop on its own | Expected in this version — automatic stop (StallGuard) is disabled. Press Stop when the curtain reaches the closed position (Section 5). |
| Motor driver gets hot | Lower the IRUN value. |

---

## 13. Quick Reference

**Web page controls**

| Control (web page) | Purpose |
| --- | --- |
| Ropener (cover) | Open, close, set position, or stop. |
| State | Shows IDLE / OPENING / CLOSING / HOMING. |
| Start-Stop Homing | Start or cancel calibration. |
| Open Time / Close Time | Daily schedule times. |
| Schedule Enabled | Turn the daily schedule on/off. |
| Timezone | Your local timezone (POSIX string). |
| Centimeters | Curtain travel distance. |
| Speed / Acceleration | Motion tuning. |
| Motor Direction | Reverse open/close direction. |
| IRUN value | Motor torque / current. |
| SGTHRS / TCOOLTHRS | Stall-detection tuning for homing. |
| SG_RESULT / TSTEP | Live readings used when tuning homing. |

**Physical buttons**

| Physical button | Action |
| --- | --- |
| Button 1 – short press | Close (or stop if moving). |
| Button 1 – long press | Start/cancel homing. |
| Button 2 – release | Open (or stop if moving). |
| Button 3 – long press | Reset Wi-Fi and reboot to setup. |

---

*Valar Systems • [valarsystems.com](https://valarsystems.com)*
