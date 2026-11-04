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

## Amendment 3 — P1.4 control and F8-P3 re-registered (appended; nothing above was edited)

A ruling of 2026-11-04 ~05:40Z, pasted into the working session by Daniel, is recorded
outside this repository at its own instruction. It re-registers P1.4's control, chooses
F8b's design (one OTA state listener per product) and re-words F8-P3, as below.

### P1.4 — the control's miss and run 1, on record

Run 1 (no reading, instrument failure): all six `esphome config` runs exited 2 with git
"Filename too long" while cloning the valar-motion package under the scratch path.
Run 2 (git long paths enabled for that process only): all six exit 0. Raw-line control:
R3000 2 changed lines (only `build_path`, moved 4 lines because the wrapper adds it
first); R3100 50 (`build_path` plus 24 `args: []` lines swapped with a neighbour); GC 84
(`build_path` plus 41 such swaps). Work stopped and was reported; no hash was read.
Diagnosis: two plain runs of identical input differ the same way (GC 86 lines, R3100
48): `esphome config` output key order is not stable between runs, so a raw diff is no
reading. Compared with keys sorted, plain vs wrapper differ only in `build_path` on all
three targets.

### P1.4 control (re-registered)

Comparison: load each resolved config with the gate's loader, dump it with keys sorted
(as the config hash is computed), unified diff. Count = the number of `+` lines (an
in-place value change gives one `-` and one `+`).
- Control: plain vs wrapper. Predicted, every target: count 1, the line being
  `build_path: build/<name>` (against `build\<name>`).
- Its own control: plain run 1 vs plain run 2. Predicted, every target: count 0.
- Its sabotage: a wrapper that also sets `api: port: 6054`. Predicted, every target:
  count 2 (`build_path` and `port`).
Then the hashes and the sabotage exactly as in Amendment 2's P1.4. Git long paths stay
enabled only for those runs (process environment), not in any git config file.

### F8b — design

Each product registers one `ota::OTAGlobalStateListener` in an `on_boot` lambda at
priority 900. On `OTA_STARTED` it issues the cover's normal STOP call
(`make_call().set_command_stop().perform()`), so Glasscalibur runs its stop_action
(motion_halt, plus cal_abort while calibrating) and Ropener runs its stop_action.
Every OTA platform reports its start through the same global callback, so one listener
covers them all. ESPHome compiles that listener interface in only when some OTA platform
has an `on_*` trigger (`USE_OTA_STATE_LISTENER`); neither base build defines it, and
nothing else in either config requests it. So the `esphome` OTA platform also gets an
`on_begin` that only logs: it is there to keep that define on. (A product `on_begin`
merges into core's esphome OTA entry; see F8-M.)

### F8-P3 (re-worded)

On the resolved config of every release target: `esphome:on_boot :: 900.0` registers the
listener (`add_global_state_listener`) and its STOP call names the product's cover; the
`esphome` OTA entry carries `on_begin`. In the generated code (`src/esphome/core/defines.h`):
`#define USE_OTA_STATE_LISTENER`. The STOP it reaches is the stop_action checked by P3.2
(Glasscalibur) and P3.1 (Ropener).
Sabotage: a copy of the resolved config with the registration lambda dropped ->
`add_global_state_listener` 0 sites (red).

F8-P5 bench (unchanged otherwise): the update is started over the ESPHome update path.

### Final gate additions (F8a, F8b, F8c)

  NEW  esphome:on_boot :: 900.0
  NEW  esphome:on_shutdown :: 600.0
  NEW  ota:esphome :: on_begin

## Amendment 4 — three OTA platforms; F8b code predictions (appended; nothing above was edited)

A ruling of 2026-11-04 ~17:05Z, pasted into the working session by Daniel, is recorded
outside this repository at its own instruction.

### Three OTA platforms, and the lesson

F8-M's resolved configs list three OTA platforms on every release target: `esphome`,
`http_request`, and `web_server` (the update upload on the device's own web page, which
ESPHome adds whenever `web_server:` is configured; Glasscalibur's firmware README tells
customers to use it). The earlier F8 analysis named two, because it read core's YAML.
Lesson, on record: any "every platform" claim is read from the resolved config (the
merged output), not from core's YAML. The one listener in F8b covers all three; F8-P3 is
unchanged.

### F8-P6 (new)

On every release target's resolved config, the OTA platforms are exactly `esphome`,
`http_request` and `web_server` — three entries, no other. Sabotage: a copy of a resolved
config with a fourth OTA entry added -> red. A new platform can then never appear
unexamined.

### F8b code (predictions committed before it)

Each product's `on_boot` at priority 900 registers one `ota::OTAGlobalStateListener`
whose `OTA_STARTED` handler issues the cover's normal STOP; the `esphome` OTA platform
gets a log-only `on_begin` (keeps `USE_OTA_STATE_LISTENER` compiled in). Predicted, every
release target: `esphome config` and `esphome compile` exit 0; F8-P3 and F8-P6 pass;
gate vs the previous commit: exactly `NEW esphome:on_boot :: 900.0` and
`NEW ota:esphome :: on_begin`.

### F8-P5 bench (path updated)

Daniel present, fixed build, Slow, mid-travel away from the ends: start an update from the
device's own web page (the path the README gives customers). Expected: the motor stops
within one main-loop pass of the update starting; the position after the reboot is the
stop point ±1 %.

## Amendment 5 — P6.1 re-registered after a miss (appended; nothing above was edited)

A ruling of 2026-11-04 ~17:50Z, pasted into the working session by Daniel, is recorded
outside this repository at its own instruction.

### The miss, on record

P6.1 (above) predicted 0 web-control URLs in Ropener's `*.md` / `*.html` at base and head.
Reading at head `392d615`: base 0 files, head 1 file — this predictions record itself
(the B5 bench step, and a quoted reviewer note naming Glasscalibur URLs). Restricted to
the product docs the count was 0. Work stopped and was reported.

### P6.1 (re-registered)

Check: the same `/(cover|switch|button|number|select|sensor|text_sensor|binary_sensor|light|event)/...`
pattern over the product docs only: `docs/`, `README.md`, `firmware/`, `flash/`.
Predicted: 0 at base and 0 at head. Sabotage as before: the same grep on Glasscalibur's
base README -> 1.

## Amendment 6 — bench plan re-registered for the session (appended; nothing above was edited)

A ruling of 2026-11-04 ~18:15Z, pasted into the working session by Daniel, is recorded
outside this repository at its own instruction; it approves this amendment. The session's
ground rules, verbatim (source: the reviewer chat, Daniel's go, 2026-11-04 ~18:05Z, pasted by
Daniel):

> Daniel will run the bench session. When you announce it:
> - propose a Pacific-time window of about an hour for both products, and let Daniel pick;
> - send him the setup ahead, in plain steps: where each unit goes, what's plugged in, and that both are on Wi-Fi;
> - you drive everything from the PC, at slow speed and away from the ends. Daniel watches and can pull the plug. Ask
>   him only yes/no questions ("did it stop?");
> - end with the release firmware back on both units, and say so.

Everything runs at the Slow preset (Glasscalibur 1200 steps/s, Ropener 1500 steps/s), driven
from the PC through each unit's web server (REST, and the web page's update upload), and
away from the ends — except calibration's own homing to the limit switches, only if Daniel
answers yes. Daniel answers yes/no only. Logs are read from the web server's event stream
(not the native API, which would hide B1). Unit addresses never appear in any record.

Asked of Daniel at the start (yes/no):
- Q1 Calibration may home to the closed end (and run its open pass to the open end)? If no:
  only B2a runs; B2b-f are recorded as not run.
- Q2 Will you press Button 1 when asked (B2e, B3 step 2)? If no: those steps are recorded
  as not run, covered by P3.1/P3.2.
- Q3 Run the old-firmware comparison steps (the sabotages of B1, B2, B3 step 1, B6 step 2,
  approved as listed)? Asked because Daniel's direction for F8 was no demonstrations of the
  old behaviour. If no: recorded as not run; those checks then rest on their predictions and
  the static checks.

B1 (Glasscalibur, no motion): head bench build, 20 min with no native-API client (the unit
   disabled in HA). Expected: Uptime >= 1200 s, no "No clients; rebooting". Motion tests run
   meanwhile over REST only; nothing reboots the unit before the reading. Sabotage (if Q3):
   v2.1.0 release build, same conditions -> "No clients; rebooting", Uptime resets at ~900 s.
B2 (Glasscalibur, Slow, starting about 10 % open so homing is short): a web STOP in the beeps;
   b web STOP in 1/3 homing; c web STOP about 10 s into the open pass; d STOP over the native
   API (the path Home Assistant uses) about 10 s into the close pass, run after B1's reading;
   e Button 1 about 10 s into the open pass (if Q2); f web Close about 10 s into the open pass.
   Expected as committed: a-e the motor stops at once and stays stopped 10 s; f no further
   open-pass launch and the window closes to the close limit; all: no "CAL: … leg at irun="
   after the command, "CAL: aborted by user", 3 low beeps, Calibration Status "Aborted by
   user - run calibration again", StallGuard Enabled ON, Vibration Stall Detection ON,
   Vibration Threshold back to its pre-run value, Motor Current ~600 mA. "At once" is read
   from Daniel's yes/no and the log order. Sabotage (if Q3): v2.1.0, cases c and f -> the
   pass re-launches ("CAL: open leg at irun=15").
B3 (Glasscalibur head bench, Slow): 1. Close from about 60 %; "BENCH: simulate stall" at about
   50 % -> reverses at once; State and cover OPENING for the whole back-off (~20 s at 1200);
   then IDLE; position about +7 % from the stall point; exactly 1 Obstruction event. 2. (if Q2)
   Button 1 during the back-off -> stops at once, IDLE. 3. simulate again ~3 s into a
   back-off -> stops at once, IDLE, a 2nd event, no second back-off. 4. simulate again ~8 s
   into a back-off -> stops at once, no reverse toward the first stall point. Step 5 and its
   sabotage are dropped (only reachable near an end; covered by P4.2 and its sabotage).
   Sabotage (if Q3): bench build at 24869f6 -> step 1 shows State IDLE while reversing.
B4: the Obstruction event shows in the web UI (both products) and in Home Assistant on the
   Ropener (the Glasscalibur is disconnected from HA for B1).
B5: as committed (both products, read-only GETs, head bench firmware).
B6 (Ropener head bench, Slow): 1. Close; simulate mid-travel -> stops at once, State IDLE,
   exactly 1 Obstruction event, position unchanged from the stop point. 2. Start-Stop Homing;
   web STOP after ~3 s; then Open -> TSTEP Sensor during the Open ~249 (374000 / 1500).
   Sabotage (if Q3): v2.9.0 release build -> ~125 (still at the 3000 homing speed).
F8-P4 (both products, head firmware, no sabotage): from about 20 % open, move toward open; when
   the motor has covered at least 20 % of travel and is still away from the end, press Restart.
   Expected: after boot, position = the stop point ±1 %. The 20 % keeps "stop point" far from
   "start point", so the check cannot pass vacuously.
F8-P5 (both products, head firmware, no sabotage): the same kind of move (at least 20 % of
   travel), then upload the release image (Glasscalibur v2.1.0, Ropener v2.9.0 for its board)
   through the device's own web page. Expected: the motor stops within one main-loop pass of the
   update starting ("Firmware update starting - stopping the motor" in the log; Daniel: "did it
   stop?"); after the reboot, position = the stop point ±1 %. This leaves both units on release
   firmware; it runs last on each unit.

## Amendment 7 — Glasscalibur bench session: Daniel's answers and speed (appended; nothing above was edited)

Directions from Daniel himself in the working session, 2026-11-04 (verbatim):

> start now. Ropener is not ready, only glasscalibur. When glasscalibur is done, then i switch to power to ropener. you can load firmware. home assistant is disabled. i can press the bytton. don't worry about old firmware comparison

> try to run it at medium or fast speed to avoid heating issues

Recorded facts: the bench Glasscalibur is a complete actuator (motor, its own travel and
end switches) on 24 V, not mounted to a window. A diagnostic slow close stopped on the
closed-end switch; a slow open stopped on its step count with the open-end switch not
reached or not registering (Daniel cannot see it; he heard no stall and is sure the
switch works). Motor reached 46.4 °C during that slow open.

Changes for the Glasscalibur session (window 12:25–13:35 PST, Daniel present):
- Speed: the **Medium** preset (2600 steps/s) instead of Slow, at Daniel's direction, to
  limit motor heating. Expected B3 back-off duration: ~9–10 s (24,500 steps at 2600),
  replacing ~20 s. No other prediction depends on the speed.
- Q2 yes (Daniel presses Button 1 for B2e and B3 step 2). Q3 no: the old-firmware
  comparison steps (sabotages of B1, B2, B3 step 1) are not run; those checks rest on their
  predictions and static checks. Q1 and calibration's full open pass (B2 d) are asked when
  B2 starts.
- A motor-temperature watch stops motion at 60 °C (Daniel's limit); no motion step starts
  above 50 °C.
- Ropener (B6, and its F8-P4 / F8-P5) runs afterwards, when Daniel moves the power to it.
