# Firmware regression gate

Two diffs between the **old** and **new** resolved firmware config. Run both
before every release. A clean `esphome compile` is necessary but **not
sufficient** — it proves the YAML is valid, not that the firmware still behaves
the same.

## Why the behaviour gate exists

v2.7.0 rewired the Ropener onto `valar-core` as a remote package. The entity
gate passed perfectly: no entity dropped, renamed, or retyped. It shipped.

But the product's `on_stall` handler had been silently replaced by
`valar-core`'s default. Two live bugs resulted:

- the device wedged in `HOMING` forever after any home (killing the button
  gestures and the schedule, both of which guard on `global_state`), and
- **every** stall zeroed the position, so a curtain snagging mid-travel
  silently redefined that point as home.

No entity name changed, so nothing caught it. The behaviour gate diffs the
*automation bodies* — `on_stall`, `on_press`/`on_click`/`on_release`, script
bodies, `on_boot`, intervals, `*_action`, lambdas — anchored to stable
identities rather than list positions.

It also caught an unrelated change in the same release: the `Motor Direction`
option string `"Reversed ↺"` had lost its glyph, which would have broken any
Home Assistant automation selecting it by value.

## Usage

```sh
# 1. Resolve both configs (from a checkout of each version)
esphome config firmware/VAL3100/Ropener-VAL3100.yml > /tmp/old-3100.yaml
esphome config firmware/VAL3100/Ropener-VAL3100.yml > /tmp/new-3100.yaml

# 2. Diff them
python tools/regression-gate/gate.py /tmp/old-3100.yaml /tmp/new-3100.yaml

# 3. Inspect anything it flags
python tools/regression-gate/gate.py /tmp/old-3100.yaml /tmp/new-3100.yaml \
    --detail on_stall
```

Exit code `0` = both gates clean, `1` = something differs. **A difference is not
automatically a failure** — additions and refactors are often intended. The
gate's job is to guarantee nothing changes *unnoticed*. Every flagged line must
be explained in the release notes.

Run it for **each board** (VAL3000 and VAL3100); they share a product layer but
resolve differently.

Requires `pyyaml` — use the interpreter ESPHome is installed under.

## Known-benign differences after the valar-core rewire

Expected when diffing a pre-rewire release (≤ v2.6.4) against a rewired one:

| Anchor | Why |
|---|---|
| 7 diagnostic entities added | `valar-core` shared diagnostics (Restart, Uptime, WiFi Signal, ESP Internal Temperature, IP/SSID/MAC) |
| `esphome:on_boot` split 600 / 400 / -100 | core / product / scheduling layers; relative order preserved |
| `script:schedule_open`, `script:schedule_close` | scheduling-mixin contract; the `global_state != 3` homing guard moved into these scripts |
| `datetime:Open Time`/`Close Time` `on_time`, `interval:60s` | now call the schedule scripts instead of inlining `cover.open`/`cover.close` |

Anything **outside** this table needs justification before release.

## Before Glasscalibur

This tool should move to `valar-motion` so both products share one copy rather
than duplicating it — same rule as the firmware itself. Glasscalibur's overrides
(limit switches, TMP1075 thermal cutoff, LIS2DH12 tamper, buzzer) carry the same
silent-replacement risk as `on_stall` did here, and there a dropped thermal
cutoff or tamper handler is a safety regression, not a UX one.
