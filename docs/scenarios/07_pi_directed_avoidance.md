# Scenario 7: Pi-Directed Avoidance via PI_AVOID Hint

## Situation
The car is driving forward. The Pi's camera sees that the right side has a drop-off (cliff edge) that the ultrasonics can't detect. The Pi sends `PI_AVOID LEFT` as a preventive hint. Later, the left ultrasonic detects a wall. Normally the car would turn right (away from the left wall), but the Pi's hint overrides: the car turns **left** instead, because the Pi knows the right side is dangerous.

## Initial State
`STATE_DRIVING` — moving forward at PWM 180.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
         (Pi camera sees cliff edge on the right side)

0ms      Pi → Arduino    PI_AVOID LEFT                        ← hint stored
0ms      Arduino → Pi    [ACK] PI_AVOID LEFT

         (Car continues driving normally)

100ms    Arduino → Pi    [TELEM] US LEFT=60.0 RIGHT=85.0     ← clear
200ms    Arduino → Pi    [TELEM] US LEFT=40.0 RIGHT=80.0     ← clear
300ms    Arduino → Pi    [TELEM] US LEFT=15.0 RIGHT=78.0     ← ⚠️ LEFT < 20cm!

         (Normally: obstacle left → turn RIGHT)
         (But PI_AVOID LEFT is stored → override to turn LEFT)

300ms    Arduino → Pi    [TELEM] AVOID LEFT                   ← Pi hint used!

  --- Phase 1: Reverse (400ms) ---
  (motors: left -150, right -150)

700ms    (internal)      Phase 1 complete

  --- Phase 2: Skid-Turn LEFT (500ms) ---
  (motors: left -150, right +150)
  (This is OPPOSITE of what the Arduino would have done locally!)

1200ms   Arduino → Pi    [TELEM] DONE AVOID

  --- Phase 3: Resume Forward (2000ms) ---
  (motors: left +180, right +180)

3200ms   Arduino → Pi    [TELEM] DONE DRIVE
3200ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors | Note |
|---|---|---|---|---|
| Reverse | 400ms | PWM 150 backward | PWM 150 backward | Back away |
| Skid-turn **LEFT** | 500ms | PWM 150 backward | PWM 150 forward | Pi hint override! |
| Resume forward | 2000ms | PWM 180 forward | PWM 180 forward | Continue |

## Without the Pi hint (default behavior)
The car would have turned **RIGHT** (away from the left wall) — directly toward the cliff edge the Pi detected. The `PI_AVOID LEFT` hint prevented this.

## Final State
`STATE_IDLE` — car avoided the wall while staying away from the cliff edge.

## Key Takeaway
- `PI_AVOID` is a **one-shot hint** — it's consumed (cleared) after one use
- It overrides the Arduino's local decision about which direction to turn
- The Pi uses this when it has information the ultrasonics don't (cliff edges, drop-offs, specific obstacles visible only to the camera)
- If no ultrasonic trigger happens, the hint just sits there until the next trigger or until overwritten by a new `PI_AVOID`
