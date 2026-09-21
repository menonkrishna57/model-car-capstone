# Scenario 3: Wall on the Right — Local Ultrasonic Avoidance

## Situation
The car is driving forward. A wall or object appears on the right side, within 20cm of the right HC-SR04P sensor. Mirror image of Scenario 2.

## Initial State
`STATE_DRIVING` — moving forward at PWM 180.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Arduino → Pi    [TELEM] US LEFT=82.0 RIGHT=55.0     ← still clear
100ms    Arduino → Pi    [TELEM] US LEFT=80.5 RIGHT=30.2     ← getting closer
200ms    Arduino → Pi    [TELEM] US LEFT=79.0 RIGHT=12.4     ← ⚠️ RIGHT < 20cm!
200ms    Arduino → Pi    [TELEM] AVOID LEFT                   ← avoidance starts

  --- Phase 1: Reverse (400ms) ---
  (motors: left -150, right -150)

600ms    (internal)      Phase 1 complete

  --- Phase 2: Skid-Turn Left (500ms) ---
  (motors: left -150, right +150)

1100ms   Arduino → Pi    [TELEM] DONE AVOID

  --- Phase 3: Resume Forward (2000ms) ---
  (motors: left +180, right +180)

3100ms   Arduino → Pi    [TELEM] DONE DRIVE
3100ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Why |
|---|---|---|---|---|
| Reverse | 400ms | PWM 150 backward | PWM 150 backward | Back away from obstacle |
| Skid-turn left | 500ms | PWM 150 backward | PWM 150 forward | Turn away from right wall |
| Resume forward | 2000ms | PWM 180 forward | PWM 180 forward | Continue original path |

## Final State
`STATE_IDLE` — obstacle cleared.

## Key Takeaway
- Obstacle on right → car turns **left** (away from obstacle)
- Exact mirror of Scenario 2
