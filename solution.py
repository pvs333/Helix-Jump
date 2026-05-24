#!/usr/bin/env python3
"""
Autonomous drone fleet routing engine.

Reads a JSON problem description from stdin and writes a flight manifest
JSON to stdout that maximizes:

    score = on_time_deliveries * 100 - total_energy * 0.1 - makespan * 0.05

The planner builds multi-package round trips per drone using a greedy
deadline-prioritized add + best-permutation routing strategy. It handles
no-fly zones by waiting for them to deactivate when blocked and inserts a
charging-station stop before the final return when the on-board battery is
not enough to make it home.
"""

# Start of HEAD
import json
import sys
import math

input_data = json.loads(sys.stdin.read())

map_size = input_data['map_size']
warehouse = [map_size[0] / 2, map_size[1] / 2]
drones = input_data['drones']
deliveries = input_data['deliveries']
no_fly_zones = input_data.get('no_fly_zones', [])
charging_stations = input_data.get('charging_stations', [])
# End of HEAD

# Start of BODY
from itertools import permutations

BATTERY_CAPACITY = 500.0
CHARGE_RATE = 2.0  # energy units per timestep
NFZ_EPS = 1e-2     # small slack added to the wait time after an NFZ ends
TIME_EPS = 1e-6


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _segment_blocked_by_nfz(A, tA, B, nfz):
    """If the segment from A (departure tA) -> B intersects the given NFZ
    while it is active, return a tuple (s1, s2, T_start, T_end) where s1,
    s2 are the entry/exit distances along the segment. Return None if
    the path is clear. The caller can derive the absolute block window
    (max(tA+s1, T_start), min(tA+s2, T_end)) and the soonest t' >= tA at
    which the drone may safely depart: T_end - s1 + epsilon."""
    d = _dist(A, B)
    if d < 1e-9:
        return None

    T_start = nfz['T_start']
    T_end = nfz['T_end']

    if nfz['shape'] == 'circle':
        cx, cy = nfz['center']
        r = nfz['radius']
        ux = (B[0] - A[0]) / d
        uy = (B[1] - A[1]) / d
        vx = A[0] - cx
        vy = A[1] - cy
        bc = ux * vx + uy * vy
        cc = vx * vx + vy * vy - r * r
        disc = bc * bc - cc
        if disc <= 0:
            return None
        sq = math.sqrt(disc)
        s1 = -bc - sq
        s2 = -bc + sq
        s1 = max(0.0, s1)
        s2 = min(d, s2)
        if s1 >= s2:
            return None
    elif nfz['shape'] == 'rectangle':
        (x1, y1), (x2, y2) = nfz['corners']
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)
        dx = B[0] - A[0]
        dy = B[1] - A[1]
        u1, u2 = 0.0, 1.0
        # Liang-Barsky parameters
        edges = [
            (-dx, A[0] - x_min),
            (dx, x_max - A[0]),
            (-dy, A[1] - y_min),
            (dy, y_max - A[1]),
        ]
        for p, q in edges:
            if abs(p) < 1e-12:
                if q < 0:
                    return None
            else:
                tt = q / p
                if p < 0:
                    if tt > u2:
                        return None
                    if tt > u1:
                        u1 = tt
                else:
                    if tt < u1:
                        return None
                    if tt < u2:
                        u2 = tt
        if u1 >= u2:
            return None
        s1 = u1 * d
        s2 = u2 * d
    else:
        return None

    enter_t = tA + s1
    exit_t = tA + s2
    if exit_t <= T_start or enter_t >= T_end:
        return None
    return (s1, s2, T_start, T_end)


def _travel(pos, t_start, end, battery, payload, nfzs):
    """Move from pos to end at speed 1, handling NFZs by waiting at pos.

    Returns (ok, path_steps, end_time, end_battery). path_steps may include
    WAIT actions and ends with a WAYPOINT-tagged step at `end`. The caller
    is expected to retag the final step (e.g. as DELIVER, CHARGE, RETURN).
    """
    t = t_start
    path = []

    if _dist(pos, end) < 1e-9:
        path.append({'x': end[0], 'y': end[1], 't': t, 'action': 'WAYPOINT'})
        return True, path, t, battery

    max_iter = 2 * len(nfzs) + 5
    for _ in range(max_iter):
        # Aggregate the latest "earliest safe departure" across blocking NFZs
        wait_target = None
        for nfz in nfzs:
            block = _segment_blocked_by_nfz(pos, t, end, nfz)
            if block is None:
                continue
            s1, _s2, _ts, T_end = block
            cand = T_end - s1 + NFZ_EPS
            if wait_target is None or cand > wait_target:
                wait_target = cand

        if wait_target is None:
            d = _dist(pos, end)
            energy = d * (1.0 + payload)
            if battery + 1e-9 < energy:
                return False, None, None, None
            t_arrive = t + d
            battery -= energy
            path.append({'x': end[0], 'y': end[1], 't': t_arrive, 'action': 'WAYPOINT'})
            return True, path, t_arrive, battery

        if wait_target <= t + 1e-9:
            return False, None, None, None
        path.append({'x': pos[0], 'y': pos[1], 't': wait_target, 'action': 'WAIT'})
        t = wait_target

    return False, None, None, None


def _detour_charge(pos, t, battery, destination, payload, charging_stations, nfzs):
    """Route through the best feasible charging station to ensure enough
    energy to reach `destination` afterwards. Returns (ok, path, t, battery,
    new_pos) where new_pos is the charging station position."""
    if not charging_stations:
        return False, None, None, None, None

    best = None
    best_score = None
    for cs in charging_stations:
        cs_pos = (cs['x'], cs['y'])
        d_to_cs = _dist(pos, cs_pos)
        e_to_cs = d_to_cs * (1.0 + payload)
        if battery + 1e-9 < e_to_cs:
            continue
        battery_at_cs = battery - e_to_cs
        d_to_dest = _dist(cs_pos, destination)
        e_to_dest = d_to_dest * (1.0 + payload)
        charge_amount = max(0.0, e_to_dest - battery_at_cs)
        # Cap at battery capacity
        if battery_at_cs + charge_amount > BATTERY_CAPACITY + 1e-9:
            charge_amount = BATTERY_CAPACITY - battery_at_cs
        if battery_at_cs + charge_amount + 1e-9 < e_to_dest:
            continue
        charge_time = charge_amount / CHARGE_RATE
        score = d_to_cs + d_to_dest + charge_time
        if best_score is None or score < best_score:
            best_score = score
            best = (cs_pos, charge_amount, charge_time)

    if best is None:
        return False, None, None, None, None

    cs_pos, charge_amount, charge_time = best
    ok, leg_path, t_after, battery_after = _travel(pos, t, cs_pos, battery, payload, nfzs)
    if not ok:
        return False, None, None, None, None

    path = list(leg_path[:-1])
    path.append({'x': cs_pos[0], 'y': cs_pos[1], 't': t_after, 'action': 'CHARGE'})
    t = t_after
    battery = battery_after
    if charge_amount > 0:
        battery += charge_amount
        t += charge_time
        path.append({'x': cs_pos[0], 'y': cs_pos[1], 't': t, 'action': 'CHARGE_COMPLETE'})
    return True, path, t, battery, cs_pos


def _simulate_trip(state, drone, ordered, nfzs, warehouse, charging_stations):
    """Simulate a full PICKUP -> DELIVER... -> RETURN trip for one drone.

    Returns (ok, path_steps, end_state)."""
    pos = (warehouse[0], warehouse[1])
    t = state['time']
    battery = BATTERY_CAPACITY
    payload = sum(d['weight'] for d in ordered)
    if payload > drone['max_payload'] + 1e-9:
        return False, None, None

    path = [{
        'x': warehouse[0], 'y': warehouse[1], 't': t,
        'action': 'PICKUP',
        'delivery_ids': [d['id'] for d in ordered],
    }]

    for delivery in ordered:
        target = (delivery['x'], delivery['y'])
        # Pre-flight battery check: detour through a charger if we cannot
        # make this leg directly.
        d_target = _dist(pos, target)
        e_target = d_target * (1.0 + payload)
        if battery + 1e-9 < e_target:
            ok, charge_path, t, battery, pos = _detour_charge(
                pos, t, battery, target, payload, charging_stations, nfzs
            )
            if not ok:
                return False, None, None
            path.extend(charge_path)

        ok, leg_path, t, battery = _travel(pos, t, target, battery, payload, nfzs)
        if not ok:
            return False, None, None
        if t > delivery['deadline'] + TIME_EPS:
            return False, None, None
        for step in leg_path[:-1]:
            path.append(step)
        path.append({
            'x': target[0], 'y': target[1], 't': t,
            'action': 'DELIVER', 'delivery_id': delivery['id'],
        })
        pos = target
        payload -= delivery['weight']

    return_dist = _dist(pos, warehouse)
    return_energy = return_dist  # payload is 0
    if battery + 1e-9 < return_energy:
        ok, charge_path, t, battery, pos = _detour_charge(
            pos, t, battery, warehouse, 0.0, charging_stations, nfzs
        )
        if not ok:
            return False, None, None
        path.extend(charge_path)

    ok, leg_path, t, battery = _travel(pos, t, warehouse, battery, 0.0, nfzs)
    if not ok:
        return False, None, None
    for step in leg_path[:-1]:
        path.append(step)
    path.append({
        'x': warehouse[0], 'y': warehouse[1], 't': t,
        'action': 'RETURN',
    })

    end_state = {
        'time': t,
        'pos': (warehouse[0], warehouse[1]),
        'battery': BATTERY_CAPACITY,
    }
    return True, path, end_state


def _trip_score(end_state, ordered):
    # Lower is better: penalize end time, encourage many deliveries.
    return end_state['time'] - len(ordered) * 1000.0


def _best_ordering(deliveries_subset, state, drone, nfzs, warehouse, charging_stations):
    """Find the best ordering for a given subset of deliveries.

    For small subsets we enumerate every permutation; for larger ones we
    fall back to nearest-neighbor + a few rotations to keep cost bounded.
    """
    n = len(deliveries_subset)
    if n == 0:
        return None

    if n <= 6:
        candidates = [tuple(p) for p in permutations(deliveries_subset)]
    else:
        # Nearest-neighbor seed
        unord = list(deliveries_subset)
        ordered = []
        cur = warehouse
        while unord:
            nearest = min(unord, key=lambda d: _dist(cur, [d['x'], d['y']]))
            ordered.append(nearest)
            cur = [nearest['x'], nearest['y']]
            unord.remove(nearest)
        candidates = [tuple(ordered)]
        # Also try sorted-by-deadline order as a sanity option
        candidates.append(tuple(sorted(deliveries_subset, key=lambda d: d['deadline'])))

    best = None
    best_score = None
    for perm in candidates:
        result = _simulate_trip(state, drone, list(perm), nfzs, warehouse, charging_stations)
        if not result[0]:
            continue
        _, path, end_state = result
        score = _trip_score(end_state, perm)
        if best_score is None or score < best_score:
            best_score = score
            best = (list(perm), path, end_state)
    return best


def _build_trip(state, drone, remaining, nfzs, warehouse, charging_stations,
                max_per_trip=6):
    """Greedily assemble a feasible trip for this drone, prioritizing
    the most urgent deliveries first."""
    if not remaining:
        return None
    candidates = [d for d in remaining if d['weight'] <= drone['max_payload'] + 1e-9]
    if not candidates:
        return None

    cands = sorted(candidates, key=lambda d: (d['deadline'], d['weight']))

    selected = []
    weight = 0.0
    best_trip = None

    for cand in cands:
        if weight + cand['weight'] > drone['max_payload'] + 1e-9:
            continue
        new_sel = selected + [cand]
        result = _best_ordering(new_sel, state, drone, nfzs, warehouse, charging_stations)
        if result is None:
            continue
        ordered, _path, _end_state = result
        selected = ordered
        weight = sum(d['weight'] for d in selected)
        best_trip = result
        if len(selected) >= max_per_trip:
            break

    return best_trip


def solve(warehouse, drones, deliveries, no_fly_zones, charging_stations):
    flight_manifest = []

    drone_states = {}
    for d in drones:
        drone_states[d['id']] = {
            'time': 0.0,
            'pos': list(warehouse),
            'battery': BATTERY_CAPACITY,
            'path': [],
            'drone': d,
        }

    remaining = sorted(deliveries, key=lambda x: x['deadline'])

    while remaining:
        progress = False
        ordered_drones = sorted(drone_states.items(), key=lambda kv: kv[1]['time'])
        for did, state in ordered_drones:
            if not remaining:
                break
            drone = state['drone']
            trip_result = _build_trip(
                state, drone, remaining,
                no_fly_zones, warehouse, charging_stations,
            )
            if trip_result is None:
                continue
            ordered, path, end_state = trip_result
            state['path'].extend(path)
            state['time'] = end_state['time']
            state['pos'] = list(end_state['pos'])
            state['battery'] = end_state['battery']
            for d in ordered:
                remaining.remove(d)
            progress = True
        if not progress:
            break

    for did, state in drone_states.items():
        if state['path']:
            flight_manifest.append({
                'drone_id': did,
                'path': state['path'],
            })
    return flight_manifest
# End of BODY

# Start of TAIL
result = solve(warehouse, drones, deliveries, no_fly_zones, charging_stations)
output = {"flight_manifest": result}
print(json.dumps(output))
# End of TAIL
