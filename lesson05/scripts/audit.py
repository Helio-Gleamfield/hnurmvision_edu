"""Audit saved simulator launches/impacts against scoreboard and vision commands.

Default input: lesson05/artifacts. Uses only Python standard library.
"""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


def case(folder, scenario):
    events = [json.loads(line) for line in (folder / f"{scenario}-events.jsonl").read_text().splitlines()]
    launches = {e["shot_id"]: e for e in events if e["event"] == "launch"}
    assert len(launches) == sum(e["event"] == "launch" for e in events), "Duplicate launch IDs"
    impacts = [e for e in events if e["event"] == "impact"]
    ids = [e["shot_id"] for e in impacts]
    assert len(ids) == len(set(ids)), "Duplicate impacts"
    assert set(ids) == set(launches), "Unresolved or unlaunched projectile"
    selected = 0 if scenario == "static" else 1
    hits = 0
    flight = []
    for event in impacts:
        launch = launches[event["shot_id"]]
        assert not event["shadow"] and launch["selected"] == selected
        assert launch["mode"] == "Swept", "Authoritative collision mode changed"
        assert abs(math.dist(launch["velocity"], [0, 0, 0])-25) < .01
        assert math.dist(launch["gravity"], [0, -9.81, 0]) < 1e-4
        assert abs(launch["radius"]-.0085) < 1e-6
        assert event["t"] >= launch["t"]
        flight.append(event["t"]-launch["t"])
        hits += event["kind"] == "Armor" and event["target"] == selected
    score = json.loads((folder / f"{scenario}-score.json").read_text())
    assert score["selected"] == selected and score["running"]
    score = score["scores"][selected]
    assert score["shots"] == len(launches) and score["hits"] == hits
    assert score["damage"] == hits * 10 and score["off_target"] == 0
    assert len(launches) >= 100 and hits/len(launches) >= .90
    commands = list(csv.DictReader((folder / f"{scenario}-commands.csv").open()))
    assert len(commands) > 100
    stamps = [int(row["acquisition_ros_ns"]) for row in commands]
    assert all(a < b for a, b in zip(stamps, stamps[1:])), "Non-increasing image timestamps"
    valid = [r for r in commands if r["control"] == "1"]
    fired = [r for r in commands if r["shoot"] == "1"]
    assert valid and fired
    ages, leads = [], []
    for row in valid:
        age, tof, lead = (float(row[k]) for k in ("age_s", "flight_s", "lead_s"))
        assert 0 <= age <= .2 and tof > 0 and lead > 0
        assert abs(lead-age-tof) < .00012, "Lead must equal measured age + flight, delay=0"
        assert 2 <= int(row["iterations"]) <= 16
        assert abs(float(row["pitch_wire_deg"])+float(row["pitch_internal_rad"])*180/math.pi)<.0002
        ages.append(age)
        leads.append(lead)
    for row in fired:
        assert row["control"] == "1" and row["reason"] == "FIRE: feedback aligned"
        assert float(row["yaw_error_rad"]) < .018 and float(row["pitch_error_rad"]) < .009
    # Last two seconds inhibit fire, allowing short-range shots to resolve.
    assert all(r["shoot"] == "0" for r in commands if stamps[-1]-int(r["acquisition_ros_ns"]) < 1_500_000_000)
    return {
        "status": "PASS", "shots": len(launches), "armor_hits": hits,
        "accuracy_percent": 100*hits/len(launches), "off_target": score["off_target"],
        "damage": score["damage"], "unresolved_projectiles": 0,
        "impact_types": dict(Counter(e["kind"] for e in impacts)),
        "camera_frames": len(commands), "controlled_frames": len(valid), "fire_request_frames": len(fired),
        "measured_image_age_ms_mean": 1000*sum(ages)/len(ages),
        "lead_ms_mean": 1000*sum(leads)/len(leads),
        "actual_flight_ms_mean": 1000*sum(flight)/len(flight),
        "checks": ["Unique launches and impacts; no unresolved shots", "Independent collision events match simulator scoreboard",
                   "25m/s, g=9.81, 17mm projectile, swept collision unchanged", "Measured acquisition age plus ballistic flight; no gimbal lag in lead",
                   "World absolute angle and IO pitch sign", "Feedback-gated shooting and final projectile drain"]
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--artifacts", type=Path, default=Path(__file__).resolve().parents[1]/"artifacts")
    args = p.parse_args()
    result = {s: case(args.artifacts, s) for s in ("static", "translation")}
    (args.artifacts/"audit.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
