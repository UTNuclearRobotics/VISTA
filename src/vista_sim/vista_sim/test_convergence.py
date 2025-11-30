#!/usr/bin/env python3
"""Test script to check if vehicle converges to goal poses."""

import math
import sys
from vista_sim.simple_vehicle_sim_v2 import (
    Eta,
    Nu,
    DubinsAirplanePath,
    SimpleVehicleModel,
)


def test_goal(start: Eta, goal: Eta, test_name: str, max_steps: int = 5000):
    """Test if vehicle converges to goal."""
    print(f"\n{'='*60}")
    print(f"Test: {test_name}")
    print(f"Goal: north={goal.north:.2f}, east={goal.east:.2f}, depth={goal.depth:.2f}, yaw={goal.yaw:.2f}")
    print(f"{'='*60}")

    # Plan path
    planner = DubinsAirplanePath(turn_radius=1.0, max_pitch_deg=15.0)
    waypoints = planner.get_poses(start, goal, waypoint_spacing=1.0)
    
    if len(waypoints) < 2:
        print(f"❌ FAILED: Could not plan path ({len(waypoints)} waypoints)")
        return False

    print(f"✓ Path planned with {len(waypoints)} waypoints")

    # Vehicle model
    vehicle = SimpleVehicleModel(
        {
            "speed_time_constant": 2.0,
            "yaw_time_constant": 0.1,
            "pitch_time_constant": 1.5,
            "roll_ratio": 0.2,
            "turn_radius_m": 1.2,
            "max_acceleration_mps2": 1.0,
            "max_speed_mps": 0.5,
            "max_pitch_deg": 15.0,
            "pitch_proportional_gain": 0.5,
            "look_ahead_dist": 8.0,
            "nominal_speed": 0.25,
        },
        waypoints,
    )

    # Simulation
    eta = start
    nu = Nu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    dt = 0.1
    step = 0

    for step in range(max_steps):
        # Simulation step
        control = vehicle.calc_control_input(eta)
        eta, nu = vehicle.step(eta, nu, control, dt)

        # Distance to goal
        dist = math.sqrt(
            (eta.north - goal.north) ** 2
            + (eta.east - goal.east) ** 2
            + (eta.depth - goal.depth) ** 2
        )

        # Log every 100 steps
        if step % 100 == 0:
            print(f"Step {step:4d}: pos=({eta.north:7.2f}, {eta.east:7.2f}, {eta.depth:6.2f}), dist={dist:6.2f}m")

        # Check convergence
        if dist < 0.3:
            elapsed_time = step * dt
            print(f"\n✓ CONVERGED in {step} steps ({elapsed_time:.1f}s)")
            print(f"  Final pos: ({eta.north:.2f}, {eta.east:.2f}, {eta.depth:.2f})")
            return True

    # Failed to converge
    dist = math.sqrt(
        (eta.north - goal.north) ** 2
        + (eta.east - goal.east) ** 2
        + (eta.depth - goal.depth) ** 2
    )
    print(f"\n❌ TIMEOUT after {step} steps ({step * dt:.1f}s), dist={dist:.2f}m")
    return False


def main():
    """Run all test cases."""
    start = Eta(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    test_cases = [
        ("Straight", Eta(20.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("Left Turn", Eta(10.0, 10.0, 0.0, 0.0, 0.0, 1.57)),
        ("Right Turn", Eta(10.0, -10.0, 0.0, 0.0, 0.0, -1.57)),
        ("Close", Eta(3.0, 3.0, 0.0, 0.0, 0.0, 0.785)),
        ("Diagonal+Depth", Eta(15.0, 15.0, 5.0, 0.0, 0.0, 0.785)),
        ("Vertical Only", Eta(0.0, 0.0, 10.0, 0.0, 0.0, 1.57)),
    ]

    results = []
    for name, goal in test_cases:
        success = test_goal(start, goal, name, max_steps=5000)
        results.append((name, success))

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    for name, success in results:
        status = "✓ PASS" if success else "❌ FAIL"
        print(f"{name:20s}: {status}")
    
    passed = sum(1 for _, s in results if s)
    total = len(results)
    print(f"\n{passed}/{total} tests passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())