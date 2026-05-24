import json
import math
import sys


BATTERY_CAPACITY = 500.0
CHARGE_RATE = 2.0
EPS = 1e-7
SAFETY_EPS = 1e-5


def distance(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def point_tuple(x, y):
    return (float(x), float(y))


def step(point, t, action, **extra):
    item = {"x": float(point[0]), "y": float(point[1]), "t": round(float(t), 6), "action": action}
    item.update(extra)
    return item


def interval_overlaps(a0, a1, b0, b1):
    return not (a1 < b0 or a0 > b1)


def segment_zone_intervals(a, b, zone):
    """Return distance-offset intervals where segment a->b is inside a zone."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    vx, vy = bx - ax, by - ay
    seg_len = math.hypot(vx, vy)

    if seg_len <= EPS:
        if point_inside_zone(a, zone):
            return [(0.0, 0.0)]
        return []

    if zone.get("shape") == "circle":
        cx, cy = zone["center"]
        r = float(zone["radius"])
        fx, fy = ax - float(cx), ay - float(cy)
        aa = vx * vx + vy * vy
        bb = 2.0 * (fx * vx + fy * vy)
        cc = fx * fx + fy * fy - r * r
        disc = bb * bb - 4.0 * aa * cc
        if disc < -EPS:
            return []
        disc = max(0.0, disc)
        root = math.sqrt(disc)
        u0 = (-bb - root) / (2.0 * aa)
        u1 = (-bb + root) / (2.0 * aa)
        lo, hi = max(0.0, min(u0, u1)), min(1.0, max(u0, u1))
        if lo <= hi + EPS:
            return [(max(0.0, lo * seg_len), min(seg_len, hi * seg_len))]
        return []

    if zone.get("shape") == "rectangle":
        (x0, y0), (x1, y1) = zone["corners"]
        xmin, xmax = sorted((float(x0), float(x1)))
        ymin, ymax = sorted((float(y0), float(y1)))
        umin, umax = 0.0, 1.0
        for p, v, lo_bound, hi_bound in ((ax, vx, xmin, xmax), (ay, vy, ymin, ymax)):
            if abs(v) <= EPS:
                if p < lo_bound - EPS or p > hi_bound + EPS:
                    return []
                continue
            enter = (lo_bound - p) / v
            leave = (hi_bound - p) / v
            if enter > leave:
                enter, leave = leave, enter
            umin = max(umin, enter)
            umax = min(umax, leave)
            if umin > umax + EPS:
                return []
        return [(max(0.0, umin * seg_len), min(seg_len, umax * seg_len))]

    return []


def point_inside_zone(point, zone):
    x, y = float(point[0]), float(point[1])
    if zone.get("shape") == "circle":
        cx, cy = zone["center"]
        return math.hypot(x - float(cx), y - float(cy)) <= float(zone["radius"]) + EPS
    if zone.get("shape") == "rectangle":
        (x0, y0), (x1, y1) = zone["corners"]
        xmin, xmax = sorted((float(x0), float(x1)))
        ymin, ymax = sorted((float(y0), float(y1)))
        return xmin - EPS <= x <= xmax + EPS and ymin - EPS <= y <= ymax + EPS
    return False


def earliest_safe_departure(a, b, depart_time, no_fly_zones):
    """Wait at a until the straight segment a->b is safe at speed 1."""
    seg_len = distance(a, b)
    if seg_len <= EPS:
        return float(depart_time)

    depart = float(depart_time)
    for _ in range(100):
        required_depart = depart
        for zone in no_fly_zones:
            start = float(zone.get("T_start", 0.0))
            end = float(zone.get("T_end", 0.0))
            for inside_start, inside_end in segment_zone_intervals(a, b, zone):
                pass_start = depart + inside_start
                pass_end = depart + inside_end
                if interval_overlaps(pass_start, pass_end, start, end):
                    required_depart = max(required_depart, end - inside_start + SAFETY_EPS)
        if required_depart <= depart + EPS:
            return depart
        depart = required_depart
    return depart


def expanded_circle_waypoints(a, b, zone):
    if zone.get("shape") != "circle":
        return []
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = map(float, zone["center"])
    r = float(zone["radius"])
    seg_len = math.hypot(bx - ax, by - ay)
    if seg_len <= EPS:
        return []
    margin = max(1.0, r * 0.08)
    radius = r + margin
    ux, uy = (bx - ax) / seg_len, (by - ay) / seg_len
    px, py = -uy, ux
    candidates = []
    for sign in (1.0, -1.0):
        side = (cx + sign * px * radius, cy + sign * py * radius)
        candidates.append([side])
        # Two points give a wider berth for long chords and reduce accidental re-entry.
        ahead = (side[0] + ux * radius, side[1] + uy * radius)
        behind = (side[0] - ux * radius, side[1] - uy * radius)
        candidates.append([behind, ahead])
    return candidates


def expanded_rectangle_waypoints(zone):
    if zone.get("shape") != "rectangle":
        return []
    (x0, y0), (x1, y1) = zone["corners"]
    xmin, xmax = sorted((float(x0), float(x1)))
    ymin, ymax = sorted((float(y0), float(y1)))
    width, height = xmax - xmin, ymax - ymin
    margin = max(1.0, 0.05 * max(width, height, 1.0))
    ll = (xmin - margin, ymin - margin)
    lr = (xmax + margin, ymin - margin)
    ur = (xmax + margin, ymax + margin)
    ul = (xmin - margin, ymax + margin)
    return [[ll, lr], [lr, ur], [ur, ul], [ul, ll]]


def direct_segment_needs_wait(a, b, t, no_fly_zones):
    return earliest_safe_departure(a, b, t, no_fly_zones) > t + EPS


def leg_candidates(a, b, t, no_fly_zones):
    candidates = [[point_tuple(b[0], b[1])]]
    for zone in no_fly_zones:
        if not segment_zone_intervals(a, b, zone):
            continue
        if not direct_segment_needs_wait(a, b, t, [zone]):
            continue
        if zone.get("shape") == "circle":
            candidates.extend(path + [point_tuple(b[0], b[1])] for path in expanded_circle_waypoints(a, b, zone))
        elif zone.get("shape") == "rectangle":
            candidates.extend(path + [point_tuple(b[0], b[1])] for path in expanded_rectangle_waypoints(zone))
    return candidates


def simulate_leg(a, b, t, battery, payload, no_fly_zones, final_action, final_extra=None):
    """Choose a direct or simple detour path from a to b and append final_action at b."""
    best = None
    for candidate in leg_candidates(a, b, t, no_fly_zones):
        pos = point_tuple(a[0], a[1])
        cur_t = float(t)
        cur_battery = float(battery)
        energy = 0.0
        steps = []
        ok = True
        for idx, target in enumerate(candidate):
            depart = earliest_safe_departure(pos, target, cur_t, no_fly_zones)
            if depart > cur_t + EPS:
                steps.append(step(pos, depart, "WAIT"))
                cur_t = depart
            dist = distance(pos, target)
            leg_energy = dist * (1.0 + payload)
            cur_battery -= leg_energy
            if cur_battery < -1e-5:
                ok = False
                break
            cur_t += dist
            energy += leg_energy
            is_final = idx == len(candidate) - 1
            if is_final:
                extra = final_extra or {}
                steps.append(step(target, cur_t, final_action, **extra))
            else:
                steps.append(step(target, cur_t, "WAYPOINT"))
            pos = target
        if not ok:
            continue
        score_cost = energy * 0.1 + cur_t * 0.05
        if best is None or score_cost < best["score_cost"]:
            best = {
                "steps": steps,
                "time": cur_t,
                "battery": cur_battery,
                "energy": energy,
                "pos": point_tuple(b[0], b[1]),
                "score_cost": score_cost,
            }
    return best


def station_charge_start(reservations, slots, earliest, duration):
    if duration <= EPS:
        return float(earliest)
    slots = max(1, int(slots))
    candidates = [float(earliest)]
    candidates.extend(end for _, end in reservations if end >= earliest - EPS)
    for candidate in sorted(set(round(c, 9) for c in candidates)):
        overlaps = 0
        for start, end in reservations:
            if not (end <= candidate + EPS or start >= candidate + duration - EPS):
                overlaps += 1
        if overlaps < slots:
            return float(candidate)
    if reservations:
        return max(end for _, end in reservations)
    return float(earliest)


def simulate_return_plan(pos, t, battery, no_fly_zones, charging_stations):
    warehouse = simulate_return_plan.warehouse

    direct = simulate_leg(pos, warehouse, t, battery, 0.0, no_fly_zones, "RETURN")
    best = None
    if direct is not None:
        best = {
            "steps": direct["steps"],
            "time": direct["time"],
            "battery": direct["battery"],
            "energy": direct["energy"],
            "ends_at_warehouse": True,
            "reservations": [],
        }

    for idx, station in enumerate(charging_stations):
        station_point = point_tuple(station["x"], station["y"])
        to_station = simulate_leg(pos, station_point, t, battery, 0.0, no_fly_zones, "WAYPOINT")
        if to_station is None:
            continue

        # If returning from the station is possible on a full battery, reserve only
        # the charge actually required for that final leg.
        final_probe = simulate_leg(station_point, warehouse, to_station["time"], BATTERY_CAPACITY, 0.0, no_fly_zones, "RETURN")
        if final_probe is not None:
            needed = final_probe["energy"]
            charge_needed = max(0.0, min(BATTERY_CAPACITY, needed) - to_station["battery"])
            duration = charge_needed / CHARGE_RATE
            reservations = station.setdefault("_reservations", [])
            start_charge = station_charge_start(reservations, station.get("slots", 1), to_station["time"], duration)
            charge_steps = list(to_station["steps"])
            if start_charge > to_station["time"] + EPS:
                charge_steps.append(step(station_point, start_charge, "WAIT"))
            charge_steps.append(step(station_point, start_charge, "CHARGE"))
            complete_t = start_charge + duration
            charge_steps.append(step(station_point, complete_t, "CHARGE_COMPLETE"))
            charged_battery = min(BATTERY_CAPACITY, to_station["battery"] + duration * CHARGE_RATE)
            final = simulate_leg(station_point, warehouse, complete_t, charged_battery, 0.0, no_fly_zones, "RETURN")
            if final is not None:
                steps = charge_steps + final["steps"]
                candidate = {
                    "steps": steps,
                    "time": final["time"],
                    "battery": final["battery"],
                    "energy": to_station["energy"] + final["energy"],
                    "ends_at_warehouse": True,
                    "reservations": [(idx, start_charge, complete_t)] if duration > EPS else [],
                }
                if best is None or candidate["energy"] * 0.1 + candidate["time"] * 0.05 < best["energy"] * 0.1 + best["time"] * 0.05:
                    best = candidate

        final_station = simulate_leg(pos, station_point, t, battery, 0.0, no_fly_zones, "RETURN")
        if final_station is not None:
            candidate = {
                "steps": final_station["steps"],
                "time": final_station["time"],
                "battery": final_station["battery"],
                "energy": final_station["energy"],
                "ends_at_warehouse": False,
                "reservations": [],
            }
            if best is None or len(charging_stations) == 1 and candidate["time"] < best["time"]:
                best = candidate

    return best


def simulate_trip(drone, order, start_time, no_fly_zones, charging_stations):
    total_payload = sum(float(d["weight"]) for d in order)
    if total_payload > float(drone["max_payload"]) + EPS:
        return None

    warehouse = simulate_return_plan.warehouse
    t = float(start_time)
    payload = total_payload
    battery = BATTERY_CAPACITY
    energy = 0.0
    pos = warehouse
    path = [step(warehouse, t, "PICKUP", delivery_ids=[d["id"] for d in order])]

    for delivery in order:
        target = point_tuple(delivery["x"], delivery["y"])
        move = simulate_leg(pos, target, t, battery, payload, no_fly_zones, "DELIVER", {"delivery_id": delivery["id"]})
        if move is None:
            return None
        if move["time"] > float(delivery["deadline"]) + 1e-5:
            return None
        path.extend(move["steps"])
        t = move["time"]
        battery = move["battery"]
        energy += move["energy"]
        payload -= float(delivery["weight"])
        pos = target

    ret = simulate_return_plan(pos, t, battery, no_fly_zones, charging_stations)
    if ret is None:
        return None

    path.extend(ret["steps"])
    energy += ret["energy"]
    local_score = len(order) * 100.0 - energy * 0.1 - max(0.0, ret["time"] - start_time) * 0.05
    if local_score <= 0.0:
        return None
    return {
        "path": path,
        "delivery_ids": [d["id"] for d in order],
        "time": ret["time"],
        "energy": energy,
        "score": local_score,
        "ends_at_warehouse": ret["ends_at_warehouse"],
        "reservations": ret["reservations"],
    }


def delivery_sort_key(warehouse, delivery):
    return (float(delivery["deadline"]), distance(warehouse, (delivery["x"], delivery["y"])), -float(delivery["weight"]))


def candidate_pool(warehouse, pending, drone):
    feasible_weight = [d for d in pending if float(d["weight"]) <= float(drone["max_payload"]) + EPS]
    by_deadline = sorted(feasible_weight, key=lambda d: delivery_sort_key(warehouse, d))[:30]
    by_distance = sorted(feasible_weight, key=lambda d: distance(warehouse, (d["x"], d["y"])))[:20]
    seen = set()
    pool = []
    for d in by_deadline + by_distance:
        if d["id"] not in seen:
            pool.append(d)
            seen.add(d["id"])
    return pool


def build_trip_for_drone(drone, pending, start_time, no_fly_zones, charging_stations):
    warehouse = simulate_return_plan.warehouse
    best_trip = None
    for seed in candidate_pool(warehouse, pending, drone):
        route = [seed]
        route_ids = {seed["id"]}
        current = simulate_trip(drone, route, start_time, no_fly_zones, charging_stations)
        if current is None:
            continue

        while True:
            used_weight = sum(float(d["weight"]) for d in route)
            remaining = [
                d for d in pending
                if d["id"] not in route_ids and used_weight + float(d["weight"]) <= float(drone["max_payload"]) + EPS
            ]
            remaining = sorted(
                remaining,
                key=lambda d: (
                    float(d["deadline"]),
                    distance((route[-1]["x"], route[-1]["y"]), (d["x"], d["y"])),
                    -float(d["weight"]),
                ),
            )[:35]
            best_extension = None
            for delivery in remaining:
                for pos in range(len(route) + 1):
                    trial = route[:pos] + [delivery] + route[pos:]
                    result = simulate_trip(drone, trial, start_time, no_fly_zones, charging_stations)
                    if result is None:
                        continue
                    if best_extension is None or result["score"] > best_extension["result"]["score"] + EPS:
                        best_extension = {"route": trial, "result": result, "added": delivery}
            if best_extension is None or best_extension["result"]["score"] <= current["score"] + EPS:
                break
            route = best_extension["route"]
            route_ids.add(best_extension["added"]["id"])
            current = best_extension["result"]

        if best_trip is None:
            best_trip = current
        elif (len(current["delivery_ids"]), current["score"], -current["time"]) > (
            len(best_trip["delivery_ids"]),
            best_trip["score"],
            -best_trip["time"],
        ):
            best_trip = current

    return best_trip


def commit_reservations(charging_stations, reservations):
    for idx, start, end in reservations:
        charging_stations[idx].setdefault("_reservations", []).append((start, end))
        charging_stations[idx]["_reservations"].sort()


def solve(warehouse, drones, deliveries, no_fly_zones, charging_stations):
    """
    Schedule drone deliveries to maximize on-time deliveries while respecting
    dynamic no-fly zones, payload limits, battery capacity, and charging.
    """
    simulate_return_plan.warehouse = point_tuple(warehouse[0], warehouse[1])
    stations = []
    for station in charging_stations:
        stations.append({
            "x": float(station["x"]),
            "y": float(station["y"]),
            "slots": int(station.get("slots", 1)),
            "_reservations": [],
        })

    pending = sorted([dict(d) for d in deliveries], key=lambda d: delivery_sort_key(simulate_return_plan.warehouse, d))
    drone_states = [
        {"drone": drone, "time": 0.0, "path": [], "active": True}
        for drone in drones
    ]

    while pending and any(state["active"] for state in drone_states):
        state = min(
            (s for s in drone_states if s["active"]),
            key=lambda s: (s["time"], str(s["drone"].get("id", ""))),
        )
        trip = build_trip_for_drone(state["drone"], pending, state["time"], no_fly_zones, stations)
        if trip is None:
            state["active"] = False
            continue

        if state["path"]:
            state["path"].extend(trip["path"])
        else:
            state["path"] = trip["path"]
        state["time"] = trip["time"]
        state["active"] = bool(trip["ends_at_warehouse"])
        commit_reservations(stations, trip["reservations"])
        delivered = set(trip["delivery_ids"])
        pending = [d for d in pending if d["id"] not in delivered]

    flight_manifest = []
    for state in drone_states:
        if state["path"]:
            flight_manifest.append({"drone_id": state["drone"]["id"], "path": state["path"]})
    return flight_manifest


def main():
    input_data = json.loads(sys.stdin.read())

    map_size = input_data["map_size"]
    warehouse = [map_size[0] / 2, map_size[1] / 2]
    drones = input_data["drones"]
    deliveries = input_data["deliveries"]
    no_fly_zones = input_data.get("no_fly_zones", [])
    charging_stations = input_data.get("charging_stations", [])

    result = solve(warehouse, drones, deliveries, no_fly_zones, charging_stations)
    output = {"flight_manifest": result}
    print(json.dumps(output, separators=(",", ":")))


if __name__ == "__main__":
    main()
