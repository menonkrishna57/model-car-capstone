# Scenario 2: Wall on the Left — Local Ultrasonic Avoidance

## Situation
The car is driving forward. A wall or object appears on the left side, within 20cm of the left HC-SR04P sensor. The Pi has not sent any commands — the Arduino handles this entirely on its own as a reflex.

## Initial State
`STATE_DRIVING` — moving forward at PWM 180.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Arduino → Pi    [TELEM] US LEFT=45.0 RIGHT=80.0     ← still clear
100ms    Arduino → Pi    [TELEM] US LEFT=28.3 RIGHT=78.5     ← getting closer
200ms    Arduino → Pi    [TELEM] US LEFT=15.0 RIGHT=78.0     ← ⚠️ LEFT < 20cm!
200ms    Arduino → Pi    [TELEM] AVOID RIGHT                  ← avoidance starts

  --- Phase 1: Reverse (400ms) ---
  (motors: left -150, right -150)

600ms    (internal)      Phase 1 complete → transition to Phase 2

  --- Phase 2: Skid-Turn Right (500ms) ---
  (motors: left +150, right -150)

1100ms   (internal)      Phase 2 complete → transition to Phase 3
1100ms   Arduino → Pi    [TELEM] DONE AVOID

  --- Phase 3: Resume Forward (2000ms) ---
  (motors: left +180, right +180 — resumes at saved speed)

3100ms   Arduino → Pi    [TELEM] DONE DRIVE
3100ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Why |
|---|---|---|---|---|
| Reverse | 400ms | PWM 150 backward | PWM 150 backward | Back away from obstacle |
| Skid-turn right | 500ms | PWM 150 forward | PWM 150 backward | Turn away from left wall |
| Resume forward | 2000ms | PWM 180 forward | PWM 180 forward | Continue original path |

## Final State
`STATE_IDLE` — car has cleared the obstacle and stopped after the resume timer.

## Key Takeaway
- Obstacle on left → car turns **right** (away from obstacle)
- The avoidance is fully autonomous — no Pi involvement needed
- The car resumes at the **original drive speed** (180), not the avoidance speed (150)
- If the Pi had previously sent `PI_AVOID LEFT`, the car would turn **left** instead (overriding the local decision)
