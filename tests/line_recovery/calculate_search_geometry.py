"""Offline search geometry calculations; never opens a serial port.

Optional pose displacement is the chassis geometric centre displacement in
its INITIAL body axes (x forward, y left), with left-positive yaw in degrees.
Two poses estimate a finite-motion equivalent ICR, not an instantaneous ICR
when the actual pivot moves within the measurement interval.
"""
import argparse
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


def literal(path, name):
    text = (ROOT / path).read_text(encoding="utf-8-sig")
    match = re.search(r"^#define\s+" + re.escape(name) + r"\s+(\d+)[LU]*\s*$",
                      text, re.MULTILINE)
    if not match:
        raise ValueError(f"Cannot find literal {name} in {path}")
    return int(match.group(1))


def icr_from_displacement(dx, dy, angle_deg):
    if not all(math.isfinite(v) for v in (dx, dy, angle_deg)):
        raise ValueError("Pose values must be finite")
    angle = math.radians(angle_deg)
    a, s = 1 - math.cos(angle), math.sin(angle)
    denominator = a * a + s * s
    if denominator < 1e-6:
        raise ValueError("Yaw change too close to a whole revolution/zero for ICR estimation")
    return ((a * dx - s * dy) / denominator,
            (s * dx + a * dy) / denominator)


def point_velocity(vx, vy, yaw_rad_s, x, y):
    return vx - yaw_rad_s * y, vy + yaw_rad_s * x


def self_test():
    for centre in [(0, 0), (-64.5, 0), (64.5, -64.5)]:
        for angle_deg in (-60, -20, 20, 60):
            t = math.radians(angle_deg)
            x, y = centre
            dx = (1 - math.cos(t)) * x + math.sin(t) * y
            dy = -math.sin(t) * x + (1 - math.cos(t)) * y
            recovered = icr_from_displacement(dx, dy, angle_deg)
            assert math.dist(centre, recovered) < 1e-9
    for angle in (0, 360):
        try:
            icr_from_displacement(1, 1, angle)
        except ValueError:
            pass
        else:
            raise AssertionError("Singular pose must be rejected")
    # Rear-centred turning changes lateral motion, not same-side longitudinal
    # demand. This is the constraint the former axle-speed ratio overlooked.
    w, a, b = math.radians(120), 64.5, 64.5
    for y in (-b, b):
        front = point_velocity(0, w * a, w, a, y)
        rear = point_velocity(0, w * a, w, -a, y)
        assert math.isclose(front[0], rear[0])
        assert math.isclose(front[1], w * 129)
        assert rear[1] == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--dx-mm", type=float)
    parser.add_argument("--dy-mm", type=float)
    parser.add_argument("--angle-deg", type=float)
    args = parser.parse_args()
    if args.self_test:
        self_test()
    d = literal("Core/Inc/vehicle_geometry.h", "VEHICLE_WHEEL_DIAMETER_MM")
    b = literal("Core/Inc/vehicle_geometry.h", "VEHICLE_TRACK_WIDTH_MM")
    length = literal("Core/Inc/vehicle_geometry.h", "VEHICLE_WHEELBASE_MM")
    n = literal("Core/Inc/line_search_model.h", "LINE_SEARCH_COUNTS_PER_REV")
    k = literal("Core/Inc/line_search_model.h", "LINE_SEARCH_SKID_PERMILLE")
    yaw = literal("Core/Inc/line_search_model.h", "LINE_SEARCH_NOMINAL_YAW_MDEG_S")
    # Detect divergence from the source of these provisional calibration values.
    assert n == literal("Core/Src/encoder_turn.c", "TURN_COUNTS_PER_WHEEL_REV")
    assert k == literal("Core/Src/encoder_turn.c", "TURN_SKID_COMP_PERMILLE")
    effective_b = (b * k + 500) // 1000
    denominator = 360000 * d
    q = (yaw * effective_b * n + denominator // 2) // denominator
    report = {
        "geometry_mm": {"diameter": d, "track": b, "wheelbase": length},
        "encoder_counts_per_rev": n,
        "mm_per_encoder_count": math.pi * d / n,
        "prior_skid_factor_not_newly_measured": k / 1000,
        "rounded_effective_track_mm": effective_b,
        "nominal_yaw_deg_s_not_measured": yaw / 1000,
        "ideal_no_longitudinal_slip_cps": yaw * b * n / denominator,
        "candidate_left_search_cps_M1_M2_M3_M4": [-q, -q, q, q],
        "candidate_right_search_cps_M1_M2_M3_M4": [q, q, -q, -q],
        "candidate_wheel_circumferential_mm_s": q * math.pi * d / n,
        "leg_timers_ms": {str(t): math.ceil(t * 3600 / q) for t in (250, 900, 2400)},
        "episode_watchdog_ms": 8000,
        "centre_pivot_wheel_path_radius_mm": math.hypot(length / 2, b / 2),
        "rear_pivot_front_path_radius_mm": math.hypot(length, b / 2),
        "rear_pivot_rear_path_radius_mm": b / 2,
        "rear_pivot_body_lateral_mm_s_at_nominal_yaw": math.radians(yaw / 1000) * length / 2,
        "physical_pivot_verified": False,
    }
    pose = (args.dx_mm, args.dy_mm, args.angle_deg)
    if any(v is not None for v in pose):
        if not all(v is not None for v in pose):
            parser.error("Supply all of --dx-mm, --dy-mm and --angle-deg")
        try:
            x, y = icr_from_displacement(*pose)
        except ValueError as exc:
            parser.error(str(exc))
        report["input_pose_equivalent_icr_mm"] = {"x_forward": x, "y_left": y}
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
