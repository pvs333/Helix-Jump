#!/usr/bin/env python3
"""Autonomous drone fleet routing engine.

Reads a JSON problem description from stdin and writes a flight manifest
JSON to stdout that maximizes:

    score = on_time_deliveries * 100 - total_energy * 0.1 - makespan * 0.05

Designed to scale to thousands of deliveries:
- Trip building uses an insertion heuristic over the K most urgent
  remaining deliveries instead of an O(n!) permutation search.
- A small final-permutation pass (when the trip is short) recovers the
  exact best ordering for tight cases.
- NFZ checks short-circuit on time-window mismatch and on an axis-aligned
  bounding-box test so most checks are O(1).
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
CHARGE_RATE = 2.0
NFZ_EPS = 1e-2
TIME_EPS = 1e-6


def _preprocess_nfzs(nfzs):
    """Annotate every NFZ with an axis-aligned bounding box for quick
    rejection during segment intersection checks."""
    for nfz in nfzs:
        if nfz['shape'] == 'circle':
            cx, cy = nfz['center']
            r = nfz['radius']
            nfz['_bbox'] = (cx - r, cy - r, cx + r, cy + r)
            nfz['_cx'] = cx
            nfz['_cy'] = cy
            nfz['_r'] = r
            nfz['_r2'] = r * r
        elif nfz['shape'] == 'rectangle':
            (x1, y1), (x2, y2) = nfz['corners']
            nfz['_bbox'] = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


def _segment_blocked(ax, ay, tA, bx, by, d, nfz):
    """If segment (ax,ay)->(bx,by) departing at tA hits this NFZ during
    its active window, return (s1, s2, T_start, T_end) where s1, s2 are
    the entry/exit distances along the segment. Else None."""
    T_end = nfz['T_end']
    if tA >= T_end:
        return None
    T_start = nfz['T_start']
    if tA + d <= T_start:
        return None

    bbox = nfz['_bbox']
    seg_xmin = ax if ax < bx else bx
    seg_xmax = ax if ax > bx else bx
    if bbox[2] < seg_xmin or bbox[0] > seg_xmax:
        return None
    seg_ymin = ay if ay < by else by
    seg_ymax = ay if ay > by else by
    if bbox[3] < seg_ymin or bbox[1] > seg_ymax:
        return None

    if nfz['shape'] == 'circle':
        cx = nfz['_cx']
        cy = nfz['_cy']
        r2 = nfz['_r2']
        ux = (bx - ax) / d
        uy = (by - ay) / d
        vx = ax - cx
        vy = ay - cy
        bc = ux * vx + uy * vy
        cc = vx * vx + vy * vy - r2
        disc = bc * bc - cc
        if disc <= 0:
            return None
        sq = math.sqrt(disc)
        s1 = -bc - sq
        s2 = -bc + sq
        if s1 < 0.0:
            s1 = 0.0
        if s2 > d:
            s2 = d
        if s1 >= s2:
            return None
    else:  # rectangle
        x_min, y_min, x_max, y_max = bbox
        dx = bx - ax
        dy = by - ay
        u1, u2 = 0.0, 1.0
        # Liang-Barsky
        if dx != 0.0:
            t1 = (x_min - ax) / dx
            t2 = (x_max - ax) / dx
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > u1:
                u1 = t1
            if t2 < u2:
                u2 = t2
            if u1 > u2:
                return None
        else:
            if ax < x_min or ax > x_max:
                return None
        if dy != 0.0:
            t1 = (y_min - ay) / dy
            t2 = (y_max - ay) / dy
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > u1:
                u1 = t1
            if t2 < u2:
                u2 = t2
            if u1 > u2:
                return None
        else:
            if ay < y_min or ay > y_max:
                return None
        s1 = u1 * d
        s2 = u2 * d

    enter_t = tA + s1
    exit_t = tA + s2
    if exit_t <= T_start or enter_t >= T_end:
        return None
    return (s1, s2, T_start, T_end)


def _travel(pos, t_start, end, battery, payload, nfzs):
    """Move from pos to end at speed 1, waiting at pos when an active
    NFZ blocks the path. Returns (ok, path_steps, end_time, end_battery)
    where the last step is a WAYPOINT at `end`."""
    px, py = pos[0], pos[1]
    ex, ey = end[0], end[1]
    d = math.hypot(ex - px, ey - py)
    path = []
    t = t_start

    if d < 1e-9:
        path.append({'x': ex, 'y': ey, 't': t, 'action': 'WAYPOINT'})
        return True, path, t, battery

    if not nfzs:
        energy = d * (1.0 + payload)
        if battery + 1e-9 < energy:
            return False, None, None, None
        t_arrive = t + d
        battery -= energy
        path.append({'x': ex, 'y': ey, 't': t_arrive, 'action': 'WAYPOINT'})
        return True, path, t_arrive, battery

    max_iter = 2 * len(nfzs) + 5
    for _ in range(max_iter):
        wait_target = None
        for nfz in nfzs:
            block = _segment_blocked(px, py, t, ex, ey, d, nfz)
            if block is None:
                continue
            s1, _s2, _ts, T_end = block
            cand = T_end - s1 + NFZ_EPS
            if wait_target is None or cand > wait_target:
                wait_target = cand

        if wait_target is None:
            energy = d * (1.0 + payload)
            if battery + 1e-9 < energy:
                return False, None, None, None
            t_arrive = t + d
            battery -= energy
            path.append({'x': ex, 'y': ey, 't': t_arrive, 'action': 'WAYPOINT'})
            return True, path, t_arrive, battery

        if wait_target <= t + 1e-9:
            return False, None, None, None
        path.append({'x': px, 'y': py, 't': wait_target, 'action': 'WAIT'})
        t = wait_target

    return False, None, None, None


def _detour_charge(pos, t, battery, destination, payload, charging_stations, nfzs):
    """Route through the best feasible charging station to ensure enough
    energy to reach `destination` afterwards."""
    if not charging_stations:
        return False, None, None, None, None

    px, py = pos[0], pos[1]
    dx, dy = destination[0], destination[1]
    pf = 1.0 + payload

    best = None
    best_score = None
    for cs in charging_stations:
        cx, cy = cs['x'], cs['y']
        d_to_cs = math.hypot(cx - px, cy - py)
        e_to_cs = d_to_cs * pf
        if battery + 1e-9 < e_to_cs:
            continue
        battery_at_cs = battery - e_to_cs
        d_to_dest = math.hypot(dx - cx, dy - cy)
        e_to_dest = d_to_dest * pf
        charge_amount = e_to_dest - battery_at_cs
        if charge_amount < 0.0:
            charge_amount = 0.0
        if battery_at_cs + charge_amount > BATTERY_CAPACITY + 1e-9:
            charge_amount = BATTERY_CAPACITY - battery_at_cs
        if battery_at_cs + charge_amount + 1e-9 < e_to_dest:
            continue
        charge_time = charge_amount / CHARGE_RATE
        score = d_to_cs + d_to_dest + charge_time
        if best_score is None or score < best_score:
            best_score = score
            best = (cx, cy, charge_amount, charge_time)

    if best is None:
        return False, None, None, None, None

    cx, cy, charge_amount, charge_time = best
    ok, leg_path, t_after, battery_after = _travel(
        (px, py), t, (cx, cy), battery, payload, nfzs
    )
    if not ok:
        return False, None, None, None, None

    path = list(leg_path[:-1])
    path.append({'x': cx, 'y': cy, 't': t_after, 'action': 'CHARGE'})
    t = t_after
    battery = battery_after
    if charge_amount > 0:
        battery += charge_amount
        t += charge_time
        path.append({'x': cx, 'y': cy, 't': t, 'action': 'CHARGE_COMPLETE'})
    return True, path, t, battery, (cx, cy)


def _simulate_trip(state, drone, ordered, nfzs, warehouse, charging_stations):
    """Simulate a PICKUP -> DELIVER... -> RETURN trip. Returns
    (ok, path_steps, end_state). end_state['energy'] holds the total
    energy consumed by this trip (excluding the recharge delta)."""
    wx, wy = warehouse[0], warehouse[1]
    pos = (wx, wy)
    t = state['time']
    battery = BATTERY_CAPACITY
    payload = 0.0
    for delivery in ordered:
        payload += delivery['weight']
    if payload > drone['max_payload'] + 1e-9:
        return False, None, None

    path = [{
        'x': wx, 'y': wy, 't': t,
        'action': 'PICKUP',
        'delivery_ids': [d['id'] for d in ordered],
    }]

    energy_total = 0.0

    for delivery in ordered:
        target = (delivery['x'], delivery['y'])
        d_target = math.hypot(target[0] - pos[0], target[1] - pos[1])
        e_target = d_target * (1.0 + payload)
        if battery + 1e-9 < e_target:
            prev_batt = battery
            ok, charge_path, t, battery, pos = _detour_charge(
                pos, t, battery, target, payload, charging_stations, nfzs
            )
            if not ok:
                return False, None, None
            # Approximate energy used during detour as battery delta minus
            # the charge gained (CHARGE_COMPLETE adds to battery without
            # consuming energy).
            charge_added = 0.0
            for step in charge_path:
                if step['action'] == 'CHARGE_COMPLETE':
                    pass
            # Recompute energy via positions inline below; simpler: track
            # separately by inspecting detour path edges.
            energy_total += _path_energy(charge_path, payload, prev_pos=(pos[0], pos[1]))
            path.extend(charge_path)

        ok, leg_path, t, battery = _travel(pos, t, target, battery, payload, nfzs)
        if not ok:
            return False, None, None
        if t > delivery['deadline'] + TIME_EPS:
            return False, None, None
        # Energy used by this leg = d_target * (1 + payload)
        new_d = math.hypot(target[0] - pos[0], target[1] - pos[1])
        energy_total += new_d * (1.0 + payload)
        if leg_path:
            for step in leg_path[:-1]:
                path.append(step)
        path.append({
            'x': target[0], 'y': target[1], 't': t,
            'action': 'DELIVER', 'delivery_id': delivery['id'],
        })
        pos = target
        payload -= delivery['weight']

    return_dist = math.hypot(wx - pos[0], wy - pos[1])
    if battery + 1e-9 < return_dist:
        prev_pos = pos
        ok, charge_path, t, battery, pos = _detour_charge(
            pos, t, battery, (wx, wy), 0.0, charging_stations, nfzs
        )
        if not ok:
            return False, None, None
        energy_total += _path_energy(charge_path, 0.0, prev_pos=prev_pos)
        path.extend(charge_path)

    ok, leg_path, t, battery = _travel(pos, t, (wx, wy), battery, 0.0, nfzs)
    if not ok:
        return False, None, None
    energy_total += math.hypot(wx - pos[0], wy - pos[1])
    if leg_path:
        for step in leg_path[:-1]:
            path.append(step)
    path.append({
        'x': wx, 'y': wy, 't': t,
        'action': 'RETURN',
    })

    end_state = {
        'time': t,
        'pos': (wx, wy),
        'battery': BATTERY_CAPACITY,
        'energy': energy_total,
    }
    return True, path, end_state


def _path_energy(steps, payload, prev_pos):
    """Sum the energy cost of a sub-path starting at prev_pos. WAIT and
    CHARGE/CHARGE_COMPLETE markers contribute zero energy."""
    px, py = prev_pos[0], prev_pos[1]
    e = 0.0
    for s in steps:
        action = s['action']
        if action in ('WAIT', 'CHARGE', 'CHARGE_COMPLETE'):
            px, py = s['x'], s['y']
            continue
        d = math.hypot(s['x'] - px, s['y'] - py)
        e += d * (1.0 + payload)
        px, py = s['x'], s['y']
    return e


def _build_trip(state, drone, remaining_sorted, nfzs, warehouse,
                charging_stations, max_per_trip=6,
                candidate_urgent=60, candidate_near=40):
    """Greedy insertion trip builder.

    Selects the K most-urgent candidates that the drone can plausibly
    reach in time (ETA <= deadline), ranking them by `latest_start =
    deadline - travel_time` so that genuinely tight-but-doable deliveries
    are considered before easy ones. Each iteration picks the (candidate,
    insertion-position) combination that yields the earliest feasible
    trip end-time. A small final permutation pass refines short-trip
    orderings."""
    if not remaining_sorted:
        return None

    max_payload = drone['max_payload']
    px, py = state['pos'][0], state['pos'][1]
    cur_t = state['time']
    eps = 1e-9

    # Round-trip energy budget: with a charging stop we can roughly use
    # twice the battery capacity over a single trip; without chargers we
    # are limited to one battery's worth of energy.
    has_chargers = bool(charging_stations)
    energy_budget = BATTERY_CAPACITY * (2.0 if has_chargers else 1.0)
    wx, wy = warehouse[0], warehouse[1]

    scored = []
    for d in remaining_sorted:
        if d['weight'] > max_payload + eps:
            continue
        dist_to = math.hypot(d['x'] - px, d['y'] - py)
        if cur_t + dist_to > d['deadline'] + TIME_EPS:
            continue
        dist_back = math.hypot(d['x'] - wx, d['y'] - wy)
        round_trip_energy = dist_to * (1.0 + d['weight']) + dist_back
        if round_trip_energy > energy_budget + eps:
            continue
        latest_start = d['deadline'] - dist_to
        scored.append((latest_start, dist_to, d))

    if not scored:
        return None

    # Take the most-urgent and the nearest entries; the union gives the
    # planner a mix of "must-do-soon" and "easy-to-bundle" choices.
    by_urgency = sorted(scored, key=lambda x: x[0])
    urgent_set = by_urgency[:candidate_urgent]
    by_distance = sorted(scored, key=lambda x: x[1])
    seen = {id(d) for _ls, _dt, d in urgent_set}
    near_set = []
    for ls, dt, d in by_distance:
        if id(d) not in seen:
            near_set.append((ls, dt, d))
            seen.add(id(d))
            if len(near_set) >= candidate_near:
                break
    candidates = [d for _ls, _dt, d in urgent_set]
    candidates.extend(d for _ls, _dt, d in near_set)

    selected = []
    sel_keys = set()
    weight = 0.0
    best_trip = None
    # Track the cost (time*0.05 + energy*0.1) relative to the drone's
    # idle-at-time-cur_t baseline. Idle baseline = state['time']*0.05.
    idle_baseline = cur_t * 0.05
    prev_score = idle_baseline  # cost of "do no trip"

    while len(selected) < max_per_trip:
        best = None
        best_score = None
        n_sel = len(selected)
        for cand in candidates:
            ck = id(cand)
            if ck in sel_keys:
                continue
            if weight + cand['weight'] > max_payload + 1e-9:
                continue
            for pos in range(n_sel + 1):
                if pos == 0:
                    new_order = [cand] + selected
                elif pos == n_sel:
                    new_order = selected + [cand]
                else:
                    new_order = selected[:pos] + [cand] + selected[pos:]
                ok, path, end_state = _simulate_trip(
                    state, drone, new_order, nfzs, warehouse, charging_stations
                )
                if not ok:
                    continue
                # Mix cost and per-delivery deadline urgency: tiebreak
                # toward the candidate whose deadline slack is tightest.
                cost = end_state['time'] * 0.05 + end_state['energy'] * 0.1
                # Use the tightest deadline in the new ordering as a soft
                # urgency tiebreaker (smaller = more urgent).
                tightest = min(d['deadline'] for d in new_order)
                score = (cost, tightest)
                if best_score is None or score < best_score:
                    best_score = score
                    best = (cand, new_order, path, end_state, cost)
        if best is None:
            break
        cand, new_order, path, end_state, cost = best
        # Stop extending if the next addition costs more than ~100 score
        # points (the value of a single delivery).
        if cost - prev_score > 100.0 - 1e-3:
            break
        selected = new_order
        sel_keys.add(id(cand))
        weight += cand['weight']
        best_trip = (list(selected), path, end_state)
        prev_score = cost

    # Optional refinement: for short trips, try every permutation of the
    # selected set in case insertion got the order wrong.
    if best_trip is not None and 2 <= len(selected) <= 5:
        cur_es = best_trip[2]
        cur_best = best_trip
        cur_score = cur_es['time'] * 0.05 + cur_es['energy'] * 0.1
        ordered_now = best_trip[0]
        for perm in permutations(selected):
            perm_list = list(perm)
            if perm_list == ordered_now:
                continue
            ok, path, end_state = _simulate_trip(
                state, drone, perm_list, nfzs, warehouse, charging_stations
            )
            if not ok:
                continue
            score = end_state['time'] * 0.05 + end_state['energy'] * 0.1
            if score < cur_score:
                cur_score = score
                cur_best = (perm_list, path, end_state)
        best_trip = cur_best

    return best_trip


def solve(warehouse, drones, deliveries, no_fly_zones, charging_stations):
    flight_manifest = []

    _preprocess_nfzs(no_fly_zones)

    drone_states = {}
    for d in drones:
        drone_states[d['id']] = {
            'time': 0.0,
            'pos': list(warehouse),
            'battery': BATTERY_CAPACITY,
            'path': [],
            'drone': d,
        }

    # Sort once globally by deadline; trip-build samples its head.
    remaining = sorted(deliveries, key=lambda x: (x['deadline'], x['weight']))

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
            done_ids = {id(d) for d in ordered}
            remaining = [d for d in remaining if id(d) not in done_ids]
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
