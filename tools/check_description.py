"""Check that description/ and the BOM agree with each other, and print a summary.

Checks:
- every joint has a valid range, holds its stance angle, and is driven by exactly one servo;
- every servo's model exists in actuators.yaml and its bus exists in robot.yaml;
- the leg lengths add up to the hip axis height;
- each servo bus's sync write and sync read fit within the bus's share of the control period;
- the total mass (servos from actuators.yaml plus mass_budget.yaml) is under the hard limit with
  the add-on shells (a warning above the target without them);
- bom/bom.xlsx lists exactly as many of each servo model as robot.yaml uses (the qty column;
  spares are a column of their own).

It also prints each servo model's torque budget under robot.yaml's torque rule.

Usage:
  python tools/check_description.py
"""

import sys
from collections import Counter
from pathlib import Path

import yaml
from bom.book import read_book

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
SIDES = ("left", "right")


def expand(entries: list[dict]) -> list[dict]:
    """Expand entries whose fields contain "{side}" into a left and a right copy."""
    out = []
    for e in entries:
        if not any("{side}" in str(v) for v in e.values()):
            out.append(dict(e))
            continue
        for side in SIDES:

            def sub(v, side=side):
                if isinstance(v, str):
                    return v.replace("{side}", side)
                if isinstance(v, list):
                    return [sub(x) for x in v]
                return v

            out.append({k: sub(v) for k, v in e.items()})
    return out


def expand_buses(buses: dict) -> dict:
    out = {}
    for name, spec in buses.items():
        for side in SIDES if "{side}" in name else (None,):
            out[name.replace("{side}", side) if side else name] = spec
    return out


def bus_cycle_time(n: int, bus: dict) -> float:
    """Seconds one control cycle keeps a Feetech bus busy with n servos: one sync write of the
    goal positions and one sync read, each a USB round trip.

    Packets: header (2), id, length, instruction, address, data length, checksum (7 bytes),
    plus per servo its id and data on a write, its id on a read request; each servo replies
    with header, id, length, error, data and checksum (6 bytes plus data).
    """
    write = 7 + n * (1 + bus["write_bytes"])
    request = 7 + n
    replies = n * (6 + bus["read_bytes"])
    wire = (write + request + replies) * bus["bits_per_byte"] / bus["baud"]
    return wire + 2 * bus["usb_latency_s"] + n * bus["reply_gap_s"]


def main() -> int:
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    budget = yaml.safe_load((DESC / "mass_budget.yaml").read_text())
    errors: list[str] = []
    warnings: list[str] = []

    raw_joints = expand(robot["joints"])
    joints = {j["name"]: j for j in raw_joints}
    motors = expand(robot["motors"])
    sb = robot["servo_bus"]
    buses = expand_buses(sb["buses"])
    if len(joints) != len(raw_joints):
        errors.append("duplicate joint names")

    for name, j in joints.items():
        lo, hi = j["range"]
        if not lo < hi:
            errors.append(f"{name}: range {j['range']} is not increasing")
        key = name.removeprefix("left_").removeprefix("right_")
        if not lo <= robot["stance"].get(key, 0) <= hi:
            errors.append(f"{name}: stance angle outside its range")

    drivers: dict[str, list[str]] = {n: [] for n in joints}
    for m in motors:
        if m["model"] not in acts:
            errors.append(f"motor {m['name']}: unknown model {m['model']}")
        if m["bus"] not in buses:
            errors.append(f"motor {m['name']}: unknown bus {m['bus']}")
        elif m["model"] in acts and acts[m["model"]]["interface"] != buses[m["bus"]]["type"]:
            errors.append(f"motor {m['name']}: {m['model']} cannot run on bus {m['bus']}")
        for j in m["drives"]:
            if j not in joints:
                errors.append(f"motor {m['name']}: drives unknown joint {j}")
            else:
                drivers[j].append(m["name"])
    for j, ms in drivers.items():
        if len(ms) != 1:
            errors.append(f"joint {j}: driven by {ms or 'nothing'}")
    print(f"joints: {len(joints)}, servos: {len(motors)}")

    d = robot["dimensions"]
    legs = d["thigh"] + d["shin"] + d["ankle_height"]
    if abs(legs - d["hip_axis_height"]) > 1e-6:
        errors.append(
            f"thigh + shin + ankle = {legs:.3f} m, hip_axis_height {d['hip_axis_height']}"
        )

    # servo bus timing
    period = 1.0 / sb["rate_hz"]
    print(f"\n{'bus':10} {'servos':>6} {'cycle ms':>9} {'load':>6}   (at {sb['rate_hz']} Hz)")
    per_bus = Counter(m["bus"] for m in motors)
    for name in buses:
        n = per_bus.get(name, 0)
        t = bus_cycle_time(n, sb)
        load = t / period
        flag = "  <-- over max" if load > sb["max_load"] else ""
        print(f"{name:10} {n:6d} {t * 1e3:9.2f} {load:6.0%}{flag}")
        if load > sb["max_load"]:
            errors.append(f"bus {name}: busy {load:.0%} of the period > {sb['max_load']:.0%}")

    # torque budget per servo model under the rule, at the pack's empty voltage
    rule = robot["torque_rule"]
    v = robot["power"]["pack"][rule["at"]]
    models = Counter(m["model"] for m in motors)
    print(f"\n{'model':13} {'stall@12V':>9} {f'stall@{v}V':>10} {'peak rule':>9} {'rated':>6}  N·m")
    for k in sorted(models):
        a = acts[k]
        stall_v = a["stall_torque"] * v / a["v_rated"]
        print(
            f"{k:13} {a['stall_torque']:9.2f} {stall_v:10.2f} "
            f"{stall_v * rule['peak_share_of_stall']:9.2f} {a['rated_torque']:6.2f}"
        )

    # mass
    motor_mass = sum(acts[k]["mass"] * n for k, n in models.items() if k in acts)
    bare = sum(i["mass"] for i in budget["items"].values() if not i.get("add_on"))
    add_on = sum(i["mass"] for i in budget["items"].values() if i.get("add_on"))
    total = motor_mass + bare
    target, limit = robot["robot"]["mass_target_kg"], robot["robot"]["mass_limit_kg"]
    print(f"\n{'model':13} {'qty':>4} {'kg':>7}")
    for k, n in sorted(models.items()):
        print(f"{k:13} {n:4d} {acts[k]['mass'] * n:7.3f}")
    print(f"servos {motor_mass:.2f} kg + other {bare:.2f} kg = {total:.2f} kg bare")
    print(f"with the add-on shells ({add_on:.2f} kg): {total + add_on:.2f} kg")
    print(f"target {target:.1f} kg bare, hard limit {limit:.1f} kg with the shells")
    if total + add_on > limit:
        errors.append(f"mass {total + add_on:.2f} kg with the shells over the {limit} kg limit")
    elif total > target:
        warnings.append(f"mass {total:.2f} kg is over the {target} kg target")

    # BOM servo counts
    in_bom: Counter = Counter()
    for r in read_book()["lines"]:
        if r.get("model"):
            in_bom[r["model"]] += int(r["qty"])
    for k in sorted(set(models) | set(in_bom)):
        if models.get(k, 0) != in_bom.get(k, 0):
            errors.append(f"BOM lists {in_bom.get(k, 0)} x {k}, robot.yaml uses {models.get(k, 0)}")

    for w in warnings:
        print(f"WARNING: {w}")
    for e in errors:
        print(f"ERROR: {e}")
    if errors:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
