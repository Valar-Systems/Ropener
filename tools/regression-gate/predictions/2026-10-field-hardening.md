# Ropener field hardening — predictions

Written and committed before any code change on branch `fix/field-hardening`.
No check below had been run when this file was committed.

- Base: origin/main `a1f2b63` (firmware/ and .github/ identical to tag v2.9.0 `8b90c53`).
- Toolchain: ESPHome 2026.6.5 in an isolated venv (local Python 3.13; CI Python 3.11).
- Release targets: `firmware/VAL3000/Ropener-VAL3000.yml` ("R3000"),
  `firmware/VAL3100/Ropener-VAL3100.yml` ("R3100").
- Bench-only targets (never built by CI, never on a customer unit):
  `tools/bench/ropener-bench-VAL3000.yaml`, `tools/bench/ropener-bench-VAL3100.yaml`.

Rules. A result that differs from its prediction: stop and report; neither the
prediction nor the code is edited to make it pass. A run that produces no
reading is an instrument failure: it may be rerun, and is reported as such.
Every check has a sabotage that must turn it red; a check that cannot go red
counts as failed.

## Rulings (verbatim)

Source: the reviewer chat, 2026-10-30, pasted into the working session by Daniel.
One paragraph is omitted here at its own instruction; it is recorded outside
this repository.

> The verification is careful and the design is right: one user-STOP path, one shared abort, one shared stall response, and internal stops kept apart from the user's. Go ahead, predictions first (R5), with these rulings.
>
> R1a yes: pin both external components at the shipped commits (slimcdk b5f0e44, esphome-lis2dh12 f9cbb99), shown byte-identical (P1.6).
> R1b yes: fix gate.py first, since part of the item 3 and 4 evidence depends on it.
> R1c yes: the 5 s timer goes; the flag lives until the back-off ends.
> R1d yes.
> R1e yes.
> R1f: defer, on one condition. First state what a user Open, Close or Set-position does today in each calibration phase. If any of them can move the window with the detectors off, or start a motion STOP can't end cleanly, it comes into this PR. Otherwise it's deferred and written up as a known limitation in the calibration doc.
> R1g yes. The bench-only "Simulate stall" is never in a release target: G1 and P5.1 must show it absent from all three release builds.
> R1h yes: document only, with the follow-up named.
> R1i yes.
> R2 yes: one event per stall that stops a motion outside calibration and homing; a jam during the back-off gives a second.
> R3 yes: the real direction during the back-off, with no new state, so Home Assistant users see nothing new, only a correct state.
> R4: the B5 GETs (read-only) and the throwaway workflow_dispatch branch (deleted afterwards) are approved. The bench units, the window and the go to flash are Daniel's.
> R5 yes.
>
> Two additions to this PR:
> - Clamp GC's back-off target to [0, max travel] inside the new stall_response, with its own prediction and sabotage. It's new code anyway, and the limit switch shouldn't be the only thing that stops an over-travel.
> - Write up the position-after-any-reboot-mid-travel problem beside R1h, as a known limitation with its follow-up named (for example, boot marks the position unknown until the next end stop). It's F2's extra, and it applies to power loss, crashes and OTA, not only the 15-minute reboot.
>
> Follow-ups, recorded in this order, not in this PR:
> 1. Ropener at Fast: StallGuard can't arm during homing (TCOOLTHRS about 83 < TSTEP about 125). Say in your first report what homing does at Fast today: stops at its timeout, or keeps pushing.
> 2. R1f's three actions, if deferred.
> 3. Position after an interrupted calibration (R1h) and after a reboot mid-travel.
>
> Then the commits as you list them, one draft PR per repo. Report when CI is green on each head, and wait for approval at a named SHA. No tag or release without Daniel's go.

## Follow-up 1 — homing at Speed "Fast" today (from the code; not observed on hardware)

At Fast (9000) recompute_tcoolthrs sets TCOOLTHRS = 748000 / 9000 ≈ 83, but
home_start drives at home_speed 3000, where cruise TSTEP ≈ 374000 / 3000 ≈ 125.
StallGuard only reports while TSTEP <= TCOOLTHRS, so on_stall never fires during
homing. home_start's target is current - 100,000 steps (≈ 236 cm of rope). So it
keeps pushing: the motor drives into the closed end and slips until that target
is counted out (≈ 35 s at 3000 steps/s with the default 1500 acceleration and
3000 deceleration), then stops but stays in HOMING until home_timeout (60 s)
aborts with home NOT set. The step counter counts the slipped steps, so the
position afterwards is wrong by that amount. Pressing Start-Stop at the closed
end (the manual path) still works. With StallGuard off (the default), homing at
any speed behaves the same way unless the user presses Start-Stop.
Not changed in this PR.

## G. Instruments (run first)

G1 Entity gate with object IDs (`domain | name | entity_category | object_id`,
   object_id from ESPHome 2026.6.5's `get_base_entity_object_id`).
   gate(base, base) for R3000 and R3100. Predicted: exit 0, both gates clean.
   Sabotage: R3000 base dump with number "Motor Current" renamed "Motor Current X"
   -> exit 1; exactly 1 REMOVED `number | Motor Current | config | motor_current`
   and 1 ADDED `number | Motor Current X | config | motor_current_x`.
G2 Anchor collision fix. Ropener has one `interval: 200ms`, so the fixed gate
   names it `interval:200ms` (no ordinal). Sabotage: shared with Glasscalibur
   (the collision only exists there); see its predictions file.
G3 Call-site lister (`sites.py`). Base R3000 dump: `cover.stop` 6 sites —
   `binary_sensor:Button 1 :: on_press` ×2, `binary_sensor:Button 1 :: on_release`,
   `binary_sensor:Button 2 :: on_press` ×2, `binary_sensor:Button 2 :: on_release`.
   Sabotage: copy with one `cover.stop` added to `script:home_abort` -> 7 sites.

## 1. ESPHome pin

P1.1 After the pin commit, build-release.yml installs `esphome==2026.6.5`, no bare
   install. Sabotage: same grep on base -> bare install found.
P1.2 The CI assertion line exits 0 in the pinned venv. Sabotage: against the
   global ESPHome 2026.5.3 -> exit 1.
P1.3 Base at the pin: `esphome config` and `esphome compile` exit 0 for R3000 and
   R3100; each firmware.ota.bin contains "ESPHome version 2026.6.5 compiled on".
   Sabotage: copy with `script.execute: no_such_script` -> config exits non-zero.
P1.4 Reproduction: compile base with `-s fw_version 2.9.0`. Predicted: each build's
   config hash occurs as 4 little-endian bytes in the matching shipped v2.9.0
   Ropener-VAL3000.ota.bin / Ropener-VAL3100.ota.bin. Sabotage: the fw_version
   "dev" hash -> not found. (A miss is reported, not explained away.)
P1.5 CI green on the PR head (both matrix jobs); install log shows esphome 2026.6.5.
   Sabotage (R4-approved): workflow_dispatch on a throwaway branch with a broken
   YAML -> red; branch deleted afterwards.

## 2. reboot_timeout (already fixed in Ropener)

P2.1 main.cpp for R3000 and R3100 contains `set_reboot_timeout(0)` at base and head.
   Sabotage: copy with the product `api:` block removed -> 900000.

## 3e. STOP during homing restores the run speed (R1e)

P3.1 cover:Ropener stop_action at head contains `script.stop: home_timeout` and
   `stepper.set_speed` from global_speed. Sabotage: base dump -> absent.

## 4. Movement reported until the motor stops

P4.1 No behaviour change for item 4: a stall outside homing stops the motor and
   publishes IDLE in the same action chain, with no back-off. Check (sites.py on
   the head dump): `stepper:driver :: on_stall` has `stepper.stop` as its first
   action inside the sg_enabled branch, and `script:stall_stop` contains no
   `stepper.set_target` (0 sites). Sabotage: copy with a `stepper.set_target`
   added to stall_stop -> 1 site (red).

## 5. Obstruction event

P5.1 Entity gate head vs base, R3000 and R3100: 0 REMOVED, exactly 1 ADDED
   `event | Obstruction | (none) | obstruction`; no row containing "BENCH"
   (R1g). Sabotage: a bench target's dump vs base -> additionally ADDED
   `button | BENCH: simulate stall | …`.
P5.2 `event.trigger` 1 site — `script:stall_stop`. `script.execute stall_stop`
   1 site in each release build — `stepper:driver :: on_stall`, in the
   sg_enabled branch, not the homing branch. Sabotage: copy where the homing
   branch also calls stall_stop -> 2 sites (red).
P5.3 With StallGuard off (default) on_stall does nothing, so the event cannot fire.

## 6. Docs

P6.1 Ropener docs (*.md, *.html) contain 0 web-control URLs at base and head.
   Sabotage: the same grep on Glasscalibur's base README -> 1 hit.

## Final gate, head vs base (fixed gate on both dumps), R3000 and R3100

Entity: 0 removed, 1 added (event Obstruction).
Behaviour:
  CHANGED  cover:Ropener :: stop_action
  CHANGED  stepper:driver :: on_stall
  NEW      script:stall_stop :: then
  (2 changed, 0 dropped, 1 new.) Everything else identical.
Every commit: `esphome config` and `esphome compile` exit 0 for both targets at 2026.6.5.

## Bench plan (hardware)

Nothing here runs without Daniel present and his go for that specific test, in a
window announced in Pacific time. Bench unit only.

B5 (no motion, R4-approved; any unit on 2026.6.5) GET /cover/Ropener -> 200 JSON
   with "name_id":"cover/Ropener"; GET /cover/ropener -> 200 and a "Deprecated
   URL format" log line. Sabotage: GET /cover/NoSuchCover -> 404.
B6 (motion) Bench build.
   1. Close; "BENCH: simulate stall" mid-travel. Expected: stops at once, State
      IDLE, exactly 1 Obstruction event, position unchanged from the stop point.
   2. Speed Fast; Start-Stop Homing; web STOP; then Open. Expected: TSTEP Sensor
      during the Open ≈ 42 (cruise at 9000). Sabotage: base build -> ≈ 125
      (still at the 3000 homing speed).
B4 The Obstruction event is visible in HA and the web UI with B6's timestamp.

## Amendment 1 — P2.1 re-registered after a miss (appended; nothing above was edited)

### The miss, on record

Original P2.1 (above, unchanged): "exactly one call each". Readings at base, ESPHome
2026.6.5, generated main.cpp, all three targets:

- GC: `wifi_wificomponent_id->set_reboot_timeout(900000)` and `api_apiserver_id->set_reboot_timeout(900000)` (2 calls)
- R3000: `wifi_wificomponent_id->set_reboot_timeout(900000)` and `api_apiserver_id->set_reboot_timeout(0)` (2 calls)
- R3100: `wifi_wificomponent_id->set_reboot_timeout(900000)` and `api_apiserver_id->set_reboot_timeout(0)` (2 calls)

Cause, as recorded by the ruling: the first pattern counted every `set_reboot_timeout(`
call, and Wi-Fi's own (900000) is a second one. Work stopped and was reported.

### Ruling (verbatim; one paragraph replaced by a pointer)

Source: the reviewer chat, 2026-10-30 ~18:15Z, pasted into the working session by Daniel.

> P2.1: re-registration approved. The original text and the miss stay on record, for all three targets.
> - New P2.1: in each build's generated code, exactly one `api_apiserver_id->set_reboot_timeout(` call, with the value 0.
>   Zero matches fails as well, so an api block given its own id fails loudly instead of passing silently.
> - Its sabotage: one target built with the api block's `reboot_timeout: 0s` removed. Predicted: that target's call reads
>   900000, red. Predict, then run.
> - The miss's cause, recorded: the first pattern counted every set_reboot_timeout( call, and Wi-Fi's own (900000) is a
>   second one.
>
> P1.3: complete on all three targets, as reported. The P1.4 hashes are accepted as recorded.
>
> [One paragraph of this ruling, an observation request, is recorded outside this repository.]

### P2.1 (re-registered)

Check: in each release build's generated main.cpp, count lines matching
`api_apiserver_id->set_reboot_timeout(`. Pass = exactly one, with the value 0.
Zero matches fails; two or more fails; any other value fails.

Predicted, Ropener: R3000 and R3100 head builds each read exactly one
`api_apiserver_id->set_reboot_timeout(0)`. (The base readings above were seen before
this re-registration, so they are not counted as predicted; the fresh head builds are.)
Sabotage: R3000 built with the product layer's `reboot_timeout: 0s` line removed
(temporarily in the working tree, restored afterwards, never committed). Predicted:
exactly one `api_apiserver_id->set_reboot_timeout(900000)` -> red.

### Observation

The observation this ruling asked for is recorded outside this repository.

## Amendment 2 — 2026-11-04 rulings, P1.4 re-registered, F8 (appended; nothing above was edited)

### Rulings (verbatim; some lines replaced by pointers)

Source: the reviewer chat, pasted into the working session by Daniel.

Note, ~04:45Z:

> From the Remote edition session's reading of both products: add these to your findings if they aren't there already. No
> change without a ruling.
> - Glasscalibur: no obstruction entity. Expose an event or binary sensor; the trip counters lag by up to 30 s.
> - Glasscalibur: the wiki's API page documents /cover/glasscalibur/open, which ESPHome 2026.6.5 deprecates for removal in
>   2026.7.0. The current form is /cover/Glasscalibur/open.
> - Ropener: stall stops aren't exposed. StallGuard is off by default, and the user guide says it doesn't work in the
>   current firmware.

Ruling, ~05:00Z:

> P2.1: accepted as applied (re-registered, the sabotage red as predicted, the original and the miss kept). The head
> readings come with item 2's build.
>
> P1.4: re-registered as you proposed. The original text and the miss stay on record.
> - The check: codegen-only (--only-generate) from HEAD's committed files, with the release's fw_version and a
>   Linux-style build_path: build/<name>, set in a scratch wrapper.
> - Its control, before any hash is read: the wrapper's resolved config and a plain run's differ in exactly one line,
>   build_path. Anything else: stop.
> - Predicted: R3000's and R3100's hashes each occur exactly once in their own shipped .ota.bin. GC's (0x17807f68, once,
>   at 0x1d2a0) is already observed: report it, don't count it.
> - The sabotage as before: the dev-build hashes, found 0 times each.
> - Not the Linux CI route. It would add a workflow to a public repo for one reading, and the wrapper with its control
>   proves the same thing locally.
>
> [One line of this ruling is recorded outside this repository.]
>
> New finding, F8 (Glasscalibur): after any reboot mid-travel, the saved position is the travel's start, not where the
> motor stopped, so the position is wrong from then on. Item 2 removes the 15-minute trigger, but the Restart button, an
> OTA update or an HA restart command can still reboot it mid-travel.
> - Record it, with every path that can reboot a moving GC.
> - Propose a fix, predictions first. For example: keep the position global current during travel, as Ropener does every
>   0.2 s, so that on_shutdown saves where the motor stopped. [One sentence recorded outside this repository.]
> - No change without a ruling. Any motion test waits for Daniel to be present.
>
> Housekeeping:
> - Set the uncommitted R1a edit aside (a named stash) while P1.4 and the readings after it run, so no reading can include
>   it. Restore it when R1a's turn comes.
> - [One line recorded outside this repository.]

Rulings ~05:15Z and ~05:25Z are recorded outside this repository at their own instruction.
They approve F8a, F8b and F8c, set the order (P1.4, then item 2, F8a and F8b), and keep
F8-P4 and F8-P5 as bench checks of the fixed build with Daniel present.

### P1.4 — the miss, on record

Original P1.4 (above, unchanged). Readings, base rebuilt on Windows with the release
fw_version (every compile exit 0): GC `0x1f0d589d`, R3000 `0x75ad42d1`, R3100 `0x91f0c2a6`
— 0 occurrences each in the matching shipped .ota.bin. Sabotage (dev hashes
`0xee63201f`, `0x5ee763ff`, `0x0a59e554`): 0 each, as predicted. Work stopped and was
reported. Cause, found by diagnosis: the hashed resolved config contains
`esphome: build_path`, which Windows writes `build\<name>` and the Linux release runner
`build/<name>`. Diagnostic (not a prediction): GC codegen from HEAD's committed files
with `build_path: build/glasscalibur` gives `0x17807f68`, found once at `0x1d2a0` in the
shipped v2.1.0 Glasscalibur.ota.bin.

### P1.4 (re-registered)

Check, per target: export HEAD's committed `firmware/` to a scratch directory.
Plain run: `esphome -s fw_version <release> config <board file>`. Wrapper: a copy of the
board file with `build_path: build/<name>` added to its `esphome:` block, same command.
Control, before any hash is read: the two resolved configs differ in exactly one line,
`build_path` (`build\<name>` vs `build/<name>`); anything else, stop. Then
`compile --only-generate` on the wrapper, read `config_hash` from its build_info.json,
count its 4 little-endian bytes in the shipped .ota.bin.
Predicted, Ropener (fw_version 2.9.0), for R3000 and R3100 each: control passes
(exactly one line differs, `build_path`); the hash occurs exactly once in its own
shipped v2.9.0 .ota.bin (Ropener-VAL3000 / Ropener-VAL3100).
Sabotage: the dev-build hashes `0x5ee763ff` (R3000) and `0x0a59e554` (R3100) -> 0 each.

### F8 — recorded (from ESPHome 2026.6.5 source)

Glasscalibur saves its position only when it goes idle, so after a reboot during a
travel it restores the position the travel started from, and the position is wrong
until the window next touches an end stop. Paths that can reboot a running unit: the
API reboot timer (removed by item 2); the Restart button (web UI, HA, API); the end of
any firmware update; holding the Wi-Fi Reset button 3-10 s. Unplanned: power loss,
crash, watchdog. safe_mode acts only at boot. A soft reboot runs every component's
`on_shutdown()`: the motor stops, globals are stored and flushed, so the RAM value
survives. An unplanned reset keeps the last periodic flash sync (60 s). Ropener already
refreshes its position global every 0.2 s while moving.

### F8a — save where the motor actually stopped (both products)

`esphome: on_shutdown:` copies `id(driver)->current_position` into
`global_current_position`. Glasscalibur also refreshes `global_current_position` in the
200 ms poll while moving, as Ropener does.

Order relied on (ESPHome 2026.6.5): components are sorted highest setup priority first
(`core/application.cpp:45`, sorted at `:64-65`); shutdown walks them in reverse
(`App.reboot()` `:265-266`, `App.safe_reboot()` via `run_safe_shutdown_hooks` `:282-283`).
`on_shutdown` defaults to priority 600 (`core/config.py:307`) and fires from
`ShutdownTrigger::on_shutdown()` with that priority (`core/base_automation.h:147-148`).
Globals store on shutdown at HARDWARE 800 (`components/globals/globals_component.h:45,49`)
and the preference syncer flushes at BUS 1000 (`components/preferences/syncer.h:18-19`).
So a priority-600 `on_shutdown` runs first, the global is stored next, then flushed.

F8-P1 Resolved config, every release target: an `esphome: on_shutdown:` entry with
   priority below 800 whose lambda assigns `global_current_position` from
   `id(driver)->current_position`. Sabotage: copy with that priority set to 900 -> red.
F8-P4 Bench (motion, Daniel present, fixed build): Slow; Restart mid-travel, away from
   the ends. Expected: position after boot = the stop point ±1 %. Sabotage: base build
   -> the travel's start position (Glasscalibur).
F8-P5 Bench (motion, Daniel present, fixed build, no sabotage run): Slow; start a
   firmware update mid-travel, away from the ends. Expected: the motor stops when the
   update begins; position after reboot = the stop point ±1 %.

### F8b — pending a ruling on the merge result

F8b's implementation and F8-P3 wait for a ruling on F8-M's result.

F8-M Merge probe, per release target, scratch wrapper from HEAD's committed files:
   (a) adds `ota: - platform: esphome` with an `on_begin` logger action. Predicted:
   `esphome config` exit 0; the resolved config has exactly one `platform: esphome`
   OTA entry, and it carries `on_begin`. Sabotage: the plain resolved config -> its
   single esphome entry has no `on_begin`.
   (b) adds `ota: - platform: http_request` with an `on_begin` logger action.
   Predicted: `esphome config` exits non-zero with "Too many candidates found for"
   and "OtaHttpRequestComponent". Control: the plain config, exit 0.

### F8c — the gate sees shutdown and update hooks

G4 gate.py also anchors `esphome:on_shutdown` (key = priority) and every `ota` entry's
   `on_*` (anchor `ota:<platform>`). Predicted: gate(base, base) still clean on every
   release target. Sabotage: copy of a base dump with an `on_shutdown` entry and an
   esphome-OTA `on_begin` added -> the previous gate.py reports the behaviour gate
   clean (blind); the fixed gate reports exactly
   `NEW esphome:on_shutdown :: 600.0` and `NEW ota:esphome :: on_begin`.
F8-P2 Ropener: no change; its 200 ms poll already assigns `global_current_position`
   every tick (base, `interval:200ms :: then`).

Final gate additions (F8a, F8c), R3000 and R3100: `NEW esphome:on_shutdown :: 600.0`.
F8b rows to be added with its ruling.
