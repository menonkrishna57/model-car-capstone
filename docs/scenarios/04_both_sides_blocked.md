# Scenario 4: Both Sides Blocked — Narrow Passage

## Situation
The car is driving forward and enters a narrow gap (e.g., between two boxes, chair legs, or a doorframe). Both left AND right ultrasonics read below 20cm simultaneously.

## Initial State
`STATE_DRIVING` — moving forward at PWM 150.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Arduino → Pi    [TELEM] US LEFT=35.0 RIGHT=30.0     ← narrowing
100ms    Arduino → Pi    [TELEM] US LEFT=18.0 RIGHT=16.5     ← ⚠️ BOTH < 20cm!
100ms    Arduino → Pi    [TELEM] AVOID RIGHT                  ← default: turn right

  --- Phase 1: Reverse (400ms) ---
  (motors: left -150, right -150)

500ms    (internal)      Phase 1 complete

  --- Phase 2: Skid-Turn Right (500ms) ---
  (motors: left +150, right -150)

1000ms   Arduino → Pi    [TELEM] DONE AVOID

  --- Phase 3: Resume Forward (2000ms) ---
  (motors: left +150, right +150)

3000ms   Arduino → Pi    [TELEM] DONE DRIVE
3000ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Why |
|---|---|---|---|---|
| Reverse | 400ms | PWM 150 backward | PWM 150 backward | Back out of narrow gap |
| Skid-turn right | 500ms | PWM 150 forward | PWM 150 backward | Arbitrary default (right) |
| Resume forward | 2000ms | PWM 150 forward | PWM 150 forward | Try a new path |

## Final State
`STATE_IDLE` — car backed out and turned away.

## Key Takeaway
- When **both** sides are blocked, the Arduino defaults to turning **right** (arbitrary fallback)
- If the Pi has set `PI_AVOID LEFT` beforehand, the car will turn left instead
- This scenario may re-trigger avoidance if the car drives into another blocked area after resuming — that's expected (the car will keep trying to find a clear path)
- For a dead-end situation, the Pi's camera algorithm should detect it and take over with `PI_STOP` + manual `TURN`/`DRIVE` navigation
