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
