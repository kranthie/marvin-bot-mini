"""Print BOM totals by group from bom/bom.yaml and the quantity of each servo model; check rows.

Prices are estimates or reseller listings (see each line's price_basis). Owned items carry their
current price, so the total is the full cost to replicate the robot; the summary also gives what
is owned, ordered and still to buy, from each line's status.

Usage:
  python tools/bom/summary.py
"""

import sys
from collections import defaultdict
from pathlib import Path

import yaml

BOM = Path(__file__).resolve().parents[2] / "bom" / "bom.yaml"
FIELDS = ("item", "group", "qty", "spares", "unit_usd", "price_basis", "status")
STATUSES = ("to buy", "ordered", "owned")


def main() -> int:
    lines = yaml.safe_load(BOM.read_text())["lines"]
    errors = []
    by_group: dict[str, float] = defaultdict(float)
    by_status: dict[str, float] = defaultdict(float)
    for r in lines:
        missing = [f for f in FIELDS if f not in r]
        if missing:
            errors.append(f"{r.get('item', '?')}: missing {missing}")
            continue
        if r["status"] not in STATUSES:
            errors.append(f"{r['item']}: status {r['status']!r} not one of {STATUSES}")
        cost = (r["qty"] + r["spares"]) * r["unit_usd"]
        by_group[r["group"]] += cost
        by_status[r["status"]] += cost

    print(f"{'model':13} {'qty':>4} {'spares':>6}")
    for r in sorted((r for r in lines if r.get("model")), key=lambda r: r["model"]):
        print(f"{r['model']:13} {r['qty']:4d} {r['spares']:6d}")
    total = sum(by_group.values())
    print(f"\n{'group':12} {'USD':>7}")
    for k, v in sorted(by_group.items(), key=lambda kv: -kv[1]):
        print(f"{k:12} {v:7,.0f}")
    print(f"\ntotal {total:,.0f} USD ({len(lines)} lines, spares included), the full cost")
    print(
        f"owned {by_status['owned']:,.0f} USD, ordered {by_status['ordered']:,.0f} USD, "
        f"still to buy {by_status['to buy']:,.0f} USD"
    )
    for e in errors:
        print(f"ERROR: {e}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
