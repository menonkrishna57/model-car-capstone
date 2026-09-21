# Scenario 6: Pi Interrupts Local Avoidance Mid-Maneuver

## Situation
The car's left ultrasonic detected a wall and started a local avoidance maneuver (reversing). While the car is mid-reverse, the Pi's camera detects a person stepping into the path behind the car. The Pi sends `PI_STOP` immediately. **Tier 1 overrides Tier 2 mid-maneuver.**

## Initial State
`STATE_AVOIDING`, `AVOID_REVERSE` phase — car is currently reversing as part of a Tier 2 avoidance.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Arduino → Pi    [TELEM] US LEFT=12.0 RIGHT=75.0     ← obstacle left
0ms      Arduino → Pi    [TELEM] AVOID RIGHT                  ← avoidance started

  --- Phase 1: Reverse (started, expected to last 400ms) ---
  (motors: left -150, right -150)

         (Pi camera sees person behind the car at ~200ms)

200ms    Pi → Arduino    PI_STOP                               ← FREEZE!
200ms    Arduino → Pi    [ACK] PI_STOP
200ms    Arduino → Pi    [TELEM] STATE=PI_STOPPED

  --- Avoidance KILLED mid-phase. Motors = 0. ---
  --- Car is frozen. The remaining 200ms of reverse = cancelled. ---
  --- The skid-turn phase = cancelled. ---
  --- The resume phase = cancelled. ---

         (Person walks away)

3000ms   Pi → Arduino    PI_GO
3000ms   Arduino → Pi    [ACK] PI_GO
3000ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0

         (Clean slate. Avoidance is NOT resumed.)
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Note |
|---|---|---|---|---|
| Avoidance reverse | 200ms (of 400ms) | PWM 150 backward | PWM 150 backward | **Interrupted** |
| **FROZEN** | ~2800ms | **0** | **0** | Pi latch active |
| Idle | — | 0 | 0 | Clean slate |

## Final State
`STATE_IDLE` — the avoidance maneuver was **not resumed**. The car starts fresh. The Pi (or ultrasonics on next poll) will decide what happens next.

## Key Takeaway
- **`PI_STOP` kills everything instantly** — even a Tier 2 avoidance in progress
- The avoidance is **not resumed** after `PI_GO`. The car goes to a clean `STATE_IDLE`.
- This is a safety feature: if the Pi sees danger, the car must stop immediately regardless of what the Arduino's local logic is doing
- After `PI_GO`, the ultrasonic sensors will re-poll on the next 100ms cycle and may trigger a new avoidance if the original wall is still there
