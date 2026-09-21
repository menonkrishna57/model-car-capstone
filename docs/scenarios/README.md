# Driving Scenarios & Expected Architecture Behavior

This folder documents concrete driving scenarios and how the three-tier priority system on the Arduino Uno should react in each case. Use these as a reference for testing, debugging, and explaining the system to teammates.

## Architecture Quick Reference

```
Tier 1 — Pi Camera Latch   (PI_STOP / PI_GO)        ← highest priority
Tier 2 — Local Ultrasonic   (autonomous avoidance)
Tier 3 — Normal Commands    (DRIVE / REV / TURN)     ← lowest priority
```

**Arduino = spine** (reflexes, motor control, ultrasonics)  
**Pi 5 = brain** (camera, YOLO detection, navigation planning)

## Scenarios

| # | File | Situation | What fires |
|---|---|---|---|
| 1 | [01_clear_path.md](01_clear_path.md) | Straight driving, no obstacles | Tier 3 only |
| 2 | [02_wall_on_left.md](02_wall_on_left.md) | Wall detected by left ultrasonic | Tier 2 reflex |
| 3 | [03_wall_on_right.md](03_wall_on_right.md) | Wall detected by right ultrasonic | Tier 2 reflex |
| 4 | [04_both_sides_blocked.md](04_both_sides_blocked.md) | Both ultrasonics triggered | Tier 2 reflex |
| 5 | [05_pothole_ahead.md](05_pothole_ahead.md) | Pi camera sees pothole (no physical obstacle) | Tier 1 latch + Tier 3 navigation |
| 6 | [06_pi_stop_during_avoidance.md](06_pi_stop_during_avoidance.md) | Pi interrupts a local avoidance mid-maneuver | Tier 1 overrides Tier 2 |
| 7 | [07_pi_directed_avoidance.md](07_pi_directed_avoidance.md) | Pi recommends avoidance direction via PI_AVOID | Tier 1 hint + Tier 2 execution |
| 8 | [08_bench_test_no_pi.md](08_bench_test_no_pi.md) | Serial Monitor testing, no Pi connected | Tier 2 + Tier 3 only |
| 9 | [09_obstacle_while_reversing.md](09_obstacle_while_reversing.md) | Ultrasonic triggers during a REV command | Tier 2 reflex |
| 10 | [10_rapid_pi_commands.md](10_rapid_pi_commands.md) | Pi sends multiple commands in quick succession | Command queueing behavior |

## How to read each scenario

Each file follows this format:
1. **Situation** — What's happening in the real world
2. **Initial state** — What state the car is in when this starts
3. **Serial message flow** — Exact commands and telemetry, in order
4. **Motor behavior** — What the wheels actually do
5. **Final state** — Where the car ends up
