#!/usr/bin/env python3
"""Firmware regression gate: diff two `esphome config` dumps.

Compiling proves the YAML is valid. It does not prove the firmware still DOES
what it did. This runs two independent diffs:

  ENTITY GATE      -- every user-facing entity (domain, name, category).
                      Catches dropped/renamed/added entities, which break
                      Home Assistant entity_ids.

  BEHAVIOUR GATE   -- the automation bodies attached to those entities
                      (on_stall, on_press, script bodies, on_boot, intervals,
                      *_action, lambdas). Catches an entity that survived by
                      name while its behaviour was silently replaced.

The behaviour gate exists because of the v2.7.0 homing regression: the entity
gate passed clean while `on_stall` had been swapped for valar-core's default,
which zeroed the cover's position on every stall. Entity names alone cannot see
that class of bug.

Usage:
    python gate.py OLD.yaml NEW.yaml            # human-readable report
    python gate.py OLD.yaml NEW.yaml --detail ANCHOR_SUBSTRING

Exit code is 0 if both gates are clean, 1 if either reports a difference.
Differences are not automatically failures -- intended changes must be reviewed
and called out. The gate's job is to make sure nothing changes UNNOTICED.
"""
import argparse, json, sys, yaml

# --- shared loader -----------------------------------------------------------

DOMAINS = [
    "binary_sensor", "sensor", "text_sensor", "switch", "number", "select",
    "button", "cover", "light", "datetime", "text", "fan", "lock", "climate",
    "valve", "event", "update", "stepper", "output", "alarm_control_panel",
    "media_player",
]


class Loader(yaml.SafeLoader):
    """SafeLoader that tolerates ESPHome's custom tags (!lambda, !secret, ...)."""


def _passthrough(loader, tag_suffix, node):
    if isinstance(node, yaml.ScalarNode):
        return loader.construct_scalar(node)
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    return loader.construct_mapping(node)


Loader.add_multi_constructor("!", _passthrough)


def load(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.load(fh, Loader=Loader)


# --- entity gate -------------------------------------------------------------

def entities(cfg):
    rows = set()
    for domain in DOMAINS:
        block = cfg.get(domain)
        if not isinstance(block, list):
            continue
        for item in block:
            if not isinstance(item, dict):
                continue
            if item.get("name") is not None:
                rows.add(f"{domain}\t{item['name']}\t{item.get('entity_category') or ''}")
            # Multi-entity platforms (wifi_info, ...) nest one sub-entity per key.
            for sub in item.values():
                if isinstance(sub, dict) and "name" in sub:
                    rows.add(f"{domain}\t{sub['name']}\t{sub.get('entity_category') or ''}")
    return rows


# --- behaviour gate ----------------------------------------------------------

def is_behaviour_key(key):
    return key.startswith("on_") or key.endswith("_action") or key in (
        "lambda", "then", "condition")


def strip_comments(text):
    """Drop whole-line C++ comments. Only lines that START with // are removed --
    a trailing-comment rule would truncate URLs like https://... in literals."""
    return "\n".join(
        ln for ln in text.splitlines() if not ln.lstrip().startswith("//"))


def canon(obj):
    """Whitespace- and comment-insensitive, so reformatting or a reworded
    comment is not reported as a behaviour change."""
    if isinstance(obj, str):
        return " ".join(strip_comments(obj).split())
    if isinstance(obj, list):
        return [canon(v) for v in obj]
    if isinstance(obj, dict):
        return {k: canon(v) for k, v in sorted(obj.items())}
    return obj


def behaviours(cfg):
    """anchor -> body. Anchors are stable identities (entity name, script id,
    boot priority) rather than list positions, so a reordered package merge
    does not look like a change."""
    out = {}

    def put(anchor, key, body):
        out[(anchor, key)] = json.dumps(canon(body), sort_keys=True)

    for boot in (cfg.get("esphome") or {}).get("on_boot") or []:
        if isinstance(boot, dict):
            put("esphome:on_boot", str(boot.get("priority", "?")), boot.get("then"))

    for scr in cfg.get("script") or []:
        if isinstance(scr, dict):
            put(f"script:{scr.get('id','?')}", "then", scr.get("then"))

    for iv in cfg.get("interval") or []:
        if isinstance(iv, dict):
            put(f"interval:{iv.get('interval','?')}", "then", iv.get("then"))

    for domain in DOMAINS:
        block = cfg.get(domain)
        if not isinstance(block, list):
            continue
        for item in block:
            if not isinstance(item, dict):
                continue
            anchor = f"{domain}:{item.get('name') or item.get('id') or '?'}"
            for key, val in sorted(item.items()):
                if is_behaviour_key(key):
                    put(anchor, key, val)
                elif isinstance(val, dict):
                    for k2, v2 in sorted(val.items()):
                        if is_behaviour_key(k2):
                            put(f"{anchor}.{key}", k2, v2)
    return out


# --- reporting ---------------------------------------------------------------

def report_entities(old, new):
    removed, added = sorted(old - new), sorted(new - old)
    print("=" * 72)
    print("ENTITY GATE")
    print("=" * 72)
    if not removed and not added:
        print("  clean - entity lists identical\n")
        return True
    for row in removed:
        print(f"  REMOVED  {row}")
    for row in added:
        print(f"  ADDED    {row}")
    print(f"\n  {len(removed)} removed, {len(added)} added -- each must be intended and called out.\n")
    return False


def report_behaviours(old, new):
    changed = sorted(k for k in set(old) & set(new) if old[k] != new[k])
    dropped = sorted(set(old) - set(new))
    gained = sorted(set(new) - set(old))
    print("=" * 72)
    print("BEHAVIOUR GATE")
    print("=" * 72)
    if not changed and not dropped and not gained:
        print("  clean - all automation bodies identical\n")
        return True
    for k in changed:
        print(f"  CHANGED  {k[0]} :: {k[1]}")
    for k in dropped:
        print(f"  DROPPED  {k[0]} :: {k[1]}")
    for k in gained:
        print(f"  NEW      {k[0]} :: {k[1]}")
    print(f"\n  {len(changed)} changed, {len(dropped)} dropped, {len(gained)} new.")
    print("  Re-run with --detail <anchor> to see the bodies side by side.\n")
    return False


def detail(old, new, needle):
    for key in sorted(set(old) | set(new)):
        if needle.lower() not in f"{key[0]} {key[1]}".lower():
            continue
        o, n = old.get(key), new.get(key)
        if o == n:
            continue
        print("=" * 72)
        print(f"{key[0]}  ::  {key[1]}")
        print("-" * 30 + " OLD " + "-" * 30)
        print(json.dumps(json.loads(o), indent=2) if o else "(absent)")
        print("-" * 30 + " NEW " + "-" * 30)
        print(json.dumps(json.loads(n), indent=2) if n else "(absent)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("old")
    ap.add_argument("new")
    ap.add_argument("--detail", help="show bodies for anchors matching this substring")
    args = ap.parse_args()

    old_cfg, new_cfg = load(args.old), load(args.new)
    old_b, new_b = behaviours(old_cfg), behaviours(new_cfg)

    if args.detail:
        detail(old_b, new_b, args.detail)
        return 0

    ok_e = report_entities(entities(old_cfg), entities(new_cfg))
    ok_b = report_behaviours(old_b, new_b)
    return 0 if (ok_e and ok_b) else 1


if __name__ == "__main__":
    sys.exit(main())
