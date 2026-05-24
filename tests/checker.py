#!/usr/bin/env python3
"""Independent checker that validates a flight manifest against problem
constraints and computes the raw score.

Usage:
    python checker.py <input.json> <manifest.json>

Exits 0 on a valid manifest (prints score breakdown), 1 on validation
failure.
"""
import json
import math
import sys


SPEED = 1.0
BATTERY_CAPACITY = 500.0
CHARGE_RATE = 2.0
TIME_TOL = 1e-3
COORD_TOL = 1e-3


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_in_circle(p, c, r):
    return dist(p, c) <= r + 1e-9


def segment_in_active_nfz(A, tA, B, tB, nfz):
    """Return True if any portion of the segment from A at tA to B at tB
    intersects the NFZ during its active window."""
    d = dist(A, B)
    if d < 1e-12:
        # Stationary
        if nfz['T_start'] <= tA <= nfz['T_end'] or nfz['T_start'] <= tB <= nfz['T_end']:
            inside = _point_in_nfz(A, nfz)
            if inside:
                return True
        return False

    if nfz['shape'] == 'circle':
        cx, cy = nfz['center']
        r = nfz['radius']
        ux, uy = (B[0] - A[0]) / d, (B[1] - A[1]) / d
        vx, vy = A[0] - cx, A[1] - cy
        bc = ux * vx + uy * vy
        cc = vx * vx + vy * vy - r * r
        disc = bc * bc - cc
        if disc <= 0:
            return False
        sq = math.sqrt(disc)
        s1 = max(0.0, -bc - sq)
        s2 = min(d, -bc + sq)
        if s1 >= s2:
            return False
    elif nfz['shape'] == 'rectangle':
        (x1, y1), (x2, y2) = nfz['corners']
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)
        dx, dy = B[0] - A[0], B[1] - A[1]
        u1, u2 = 0.0, 1.0
        edges = [
            (-dx, A[0] - x_min),
            (dx, x_max - A[0]),
            (-dy, A[1] - y_min),
            (dy, y_max - A[1]),
        ]
        for p, q in edges:
            if abs(p) < 1e-12:
                if q < 0:
                    return False
            else:
                tt = q / p
                if p < 0:
                    if tt > u2:
                        return False
                    if tt > u1:
                        u1 = tt
                else:
                    if tt < u1:
                        return False
                    if tt < u2:
                        u2 = tt
        if u1 >= u2:
            return False
        s1 = u1 * d
        s2 = u2 * d
    else:
        return False

    enter_t = tA + s1
    exit_t = tA + s2
    if exit_t <= nfz['T_start'] or enter_t >= nfz['T_end']:
        return False
    return True


def _point_in_nfz(P, nfz):
    if nfz['shape'] == 'circle':
        return dist(P, nfz['center']) <= nfz['radius'] + 1e-9
    if nfz['shape'] == 'rectangle':
        (x1, y1), (x2, y2) = nfz['corners']
        return (min(x1, x2) - 1e-9 <= P[0] <= max(x1, x2) + 1e-9 and
                min(y1, y2) - 1e-9 <= P[1] <= max(y1, y2) + 1e-9)
    return False


def main():
    if len(sys.argv) != 3:
        print("usage: checker.py <input.json> <manifest.json>", file=sys.stderr)
        sys.exit(2)

    with open(sys.argv[1]) as f:
        problem = json.load(f)
    with open(sys.argv[2]) as f:
        out = json.load(f)

    map_size = problem['map_size']
    warehouse = (map_size[0] / 2.0, map_size[1] / 2.0)
    drones = {d['id']: d for d in problem['drones']}
    deliveries = {d['id']: d for d in problem['deliveries']}
    nfzs = problem.get('no_fly_zones', [])
    stations = problem.get('charging_stations', [])

    delivered = set()
    total_energy = 0.0
    makespan = 0.0
    errors = []

    manifest = out['flight_manifest']
    for entry in manifest:
        did = entry['drone_id']
        if did not in drones:
            errors.append(f"unknown drone {did}")
            continue
        max_payload = drones[did]['max_payload']
        path = entry['path']
        if not path:
            continue
        if path[0]['action'] != 'PICKUP':
            errors.append(f"{did}: path must start with PICKUP")
        if path[-1]['action'] != 'RETURN':
            errors.append(f"{did}: path must end with RETURN")

        cur_t = path[0]['t']
        cur_pos = (path[0]['x'], path[0]['y'])
        # Check pickup is at warehouse
        if abs(cur_pos[0] - warehouse[0]) > COORD_TOL or abs(cur_pos[1] - warehouse[1]) > COORD_TOL:
            errors.append(f"{did}: PICKUP not at warehouse {warehouse}, got {cur_pos}")

        battery = BATTERY_CAPACITY
        payload = 0.0

        def _pickup(step_idx, p_step):
            nonlocal payload
            for did2 in p_step.get('delivery_ids', []):
                if did2 not in deliveries:
                    errors.append(f"{did}: unknown delivery id {did2} in PICKUP")
                    continue
                payload += deliveries[did2]['weight']
            if payload > max_payload + 1e-9:
                errors.append(
                    f"{did} step {step_idx}: payload {payload} exceeds max {max_payload}"
                )

        _pickup(0, path[0])

        for i in range(1, len(path)):
            prev = path[i - 1]
            step = path[i]
            ppos = (prev['x'], prev['y'])
            spos = (step['x'], step['y'])
            d = dist(ppos, spos)
            dt = step['t'] - prev['t']
            if dt < -TIME_TOL:
                errors.append(f"{did}: time decreases at step {i}")
            action = step['action']

            if d > 1e-9:
                # Movement leg: dt must equal d / SPEED
                if abs(dt - d / SPEED) > TIME_TOL:
                    errors.append(
                        f"{did} step {i} ({action}): dt={dt:.6f} but distance={d:.6f}"
                    )
                # Energy
                energy = d * (1.0 + payload)
                total_energy += energy
                battery -= energy
                if battery < -1e-6:
                    errors.append(f"{did} step {i}: battery below 0 ({battery:.6f})")
                # NFZ collision
                for nfz in nfzs:
                    if segment_in_active_nfz(ppos, prev['t'], spos, step['t'], nfz):
                        errors.append(
                            f"{did} step {i}: passes through active NFZ"
                        )
                        break
            else:
                # Stationary step
                if action == 'CHARGE_COMPLETE':
                    # Battery gained at charge rate over dt
                    battery = min(BATTERY_CAPACITY, battery + dt * CHARGE_RATE)
                # WAIT: battery unchanged
                # CHARGE: marker for arriving at charger; no battery delta until CHARGE_COMPLETE

            # Action-specific updates
            if action == 'DELIVER':
                d_id = step.get('delivery_id')
                if d_id is None or d_id not in deliveries:
                    errors.append(f"{did} step {i}: invalid DELIVER id {d_id}")
                else:
                    if d_id in delivered:
                        errors.append(f"{did} step {i}: duplicate delivery {d_id}")
                    target = (deliveries[d_id]['x'], deliveries[d_id]['y'])
                    if abs(spos[0] - target[0]) > COORD_TOL or abs(spos[1] - target[1]) > COORD_TOL:
                        errors.append(
                            f"{did} step {i}: DELIVER pos {spos} not at delivery {target}"
                        )
                    elif step['t'] > deliveries[d_id]['deadline'] + TIME_TOL:
                        errors.append(
                            f"{did} step {i}: missed deadline for {d_id} "
                            f"({step['t']:.3f} > {deliveries[d_id]['deadline']})"
                        )
                    else:
                        delivered.add(d_id)
                    payload -= deliveries[d_id]['weight']
                    if payload < -1e-9:
                        errors.append(f"{did} step {i}: payload negative")
            elif action == 'PICKUP':
                if i != 0:
                    # Subsequent trip pickup at warehouse
                    if abs(spos[0] - warehouse[0]) > COORD_TOL or abs(spos[1] - warehouse[1]) > COORD_TOL:
                        errors.append(f"{did} step {i}: PICKUP not at warehouse")
                    if payload > 1e-9:
                        errors.append(f"{did} step {i}: PICKUP with non-zero residual payload {payload}")
                    _pickup(i, step)
            elif action == 'RETURN':
                # Recharge at warehouse
                if abs(spos[0] - warehouse[0]) > COORD_TOL or abs(spos[1] - warehouse[1]) > COORD_TOL:
                    # Allowed to return at a charging station per spec, but we
                    # still want a final stop somewhere meaningful.
                    pass
                if abs(spos[0] - warehouse[0]) <= COORD_TOL and abs(spos[1] - warehouse[1]) <= COORD_TOL:
                    battery = BATTERY_CAPACITY

            makespan = max(makespan, step['t'])
            cur_t = step['t']
            cur_pos = spos

    if errors:
        print("INVALID MANIFEST")
        for e in errors:
            print("  -", e)
        sys.exit(1)

    on_time = len(delivered)
    score = on_time * 100 - total_energy * 0.1 - makespan * 0.05
    print(f"deliveries: {on_time}/{len(deliveries)}")
    print(f"total_energy: {total_energy:.4f}")
    print(f"makespan: {makespan:.4f}")
    print(f"raw_score: {score:.4f}")


if __name__ == '__main__':
    main()
