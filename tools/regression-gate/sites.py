#!/usr/bin/env python3
"""List every place an action is called in an `esphome config` dump.

The behaviour gate says WHICH bodies changed; this says WHERE a given action
(e.g. `cover.stop`, or `script.execute` of one script) is called from, using the
same stable anchors as gate.py. It answers questions like "after the change, is
cover.stop still reachable from the calibration scripts?" without reading the
whole resolved config by eye.

Usage:
    python sites.py DUMP.yaml cover.stop
    python sites.py DUMP.yaml script.execute --id motion_halt
    python sites.py DUMP.yaml --grep "if (tgt < 0) tgt = 0;"

Prints the total and one line per anchor with its count. Requires pyyaml (and,
via gate.py's loader, nothing else).
"""
import argparse, json, sys
from collections import Counter

from gate import anchored, canon, load


def _id_of(value):
    return value.get("id") if isinstance(value, dict) else value


def count_action(node, action, want_id):
    if isinstance(node, dict):
        n = 0
        for key, val in node.items():
            if key == action and (want_id is None or _id_of(val) == want_id):
                n += 1
            n += count_action(val, action, want_id)
        return n
    if isinstance(node, list):
        return sum(count_action(v, action, want_id) for v in node)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("action", nargs="?", help="action key, e.g. cover.stop")
    ap.add_argument("--id", help="only calls whose target id is this")
    ap.add_argument("--grep", help="anchors whose body contains this text instead")
    args = ap.parse_args()
    if not args.action and not args.grep:
        ap.error("give an action or --grep TEXT")

    hits = Counter()
    for anchor, key, body in anchored(load(args.dump)):
        if args.grep:
            n = json.dumps(canon(body)).count(" ".join(args.grep.split()))
        else:
            n = count_action(body, args.action, args.id)
        if n:
            hits[f"{anchor} :: {key}"] += n

    what = args.grep and f"text {args.grep!r}" or args.action + (f" {args.id}" if args.id else "")
    print(f"{what}: {sum(hits.values())} site(s)")
    for site, n in sorted(hits.items()):
        print(f"  {n}  {site}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
