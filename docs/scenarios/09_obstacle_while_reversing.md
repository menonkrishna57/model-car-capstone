# Scenario 9: Obstacle Detected While Reversing

## Situation
The Pi told the car to reverse (e.g., backing away from a pothole). While reversing, the right ultrasonic detects an obstacle. The Arduino's reflex triggers avoidance even during a reverse command.

## Initial State
`STATE_REVERSING` — moving backward at PWM 150 (from a `REV 150 2000` command).

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Pi → Arduino    REV 150 2000
0ms      Arduino → Pi    [ACK] REV 150 2000
0ms      Arduino → Pi    [TELEM] STATE=REVERSING SPEED=150

100ms    Arduino → Pi    [TELEM] US LEFT=80.0 RIGHT=45.0     ← clear
200ms    Arduino → Pi    [TELEM] US LEFT=78.0 RIGHT=18.5     ← ⚠️ RIGHT < 20cm!
200ms    Arduino → Pi    [TELEM] AVOID LEFT                   ← avoidance triggered

  --- Phase 1: Reverse (400ms) ---
  (Note: car was ALREADY reversing, now it reverses at avoidance speed)
  (motors: left -150, right -150)

600ms    (internal)      Phase 1 complete

  --- Phase 2: Skid-Turn Left (500ms) ---
  (motors: left -150, right +150)

1100ms   Arduino → Pi    [TELEM] DONE AVOID

  --- Phase 3: Resume Forward (2000ms) ---
  (motors: left +150, right +150)
  (Note: savedDriveSpeed was 0 during REV, so uses AVOID_SPEED)

3100ms   Arduino → Pi    [TELEM] DONE DRIVE
3100ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Note |
|---|---|---|---|---|
| Original reverse | 200ms (of 2000ms) | PWM 150 backward | PWM 150 backward | **Interrupted** by ultrasonic |
| Avoidance reverse | 400ms | PWM 150 backward | PWM 150 backward | Additional reverse |
| Skid-turn left | 500ms | PWM 150 backward | PWM 150 forward | Away from right obstacle |
| Resume forward | 2000ms | PWM 150 forward | PWM 150 forward | At AVOID_SPEED (no saved drive speed from REV) |

## Final State
`STATE_IDLE` — car has cleared the obstacle.

## Key Takeaway
- Ultrasonic avoidance triggers during **any** movement state: `DRIVING`, `REVERSING`, or `TURNING`
- When the original command was `REV`, `savedDriveSpeed` is 0, so the resume phase uses `AVOID_SPEED` (150) instead
- The original `REV` command's remaining duration is lost — the car doesn't go back to reversing after avoidance
