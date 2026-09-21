# Scenario 10: Rapid Pi Commands — Queuing Behavior

## Situation
The Pi sends multiple commands in quick succession. This tests how the Arduino handles command overlap — important because the Pi's algorithm may make rapid decisions.

## Initial State
`STATE_IDLE` — motors off.

## Key Behavior
The Arduino processes commands **sequentially** via `Serial.readStringUntil('\n')`. Each new driving command **replaces** the previous one (no queue). `PI_STOP` always takes effect immediately.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────

  ── Case A: New DRIVE replaces old DRIVE ──

0ms      Pi → Arduino    DRIVE 180 5000
0ms      Arduino → Pi    [ACK] DRIVE 180 5000
         (car starts at PWM 180)

100ms    Pi → Arduino    DRIVE 120 3000                       ← new command
100ms    Arduino → Pi    [ACK] DRIVE 120 3000
         (car immediately switches to PWM 120, timer resets to 3000ms)
         (the original 5000ms command is gone)

3100ms   Arduino → Pi    [TELEM] DONE DRIVE

  ── Case B: TURN interrupts DRIVE ──

0ms      Pi → Arduino    DRIVE 180 5000
100ms    Pi → Arduino    TURN 45 150                          ← interrupts
100ms    Arduino → Pi    [ACK] TURN 45 150
         (car immediately starts turning, drive is abandoned)

550ms    Arduino → Pi    [TELEM] DONE TURN

  ── Case C: PI_STOP during rapid commands ──

0ms      Pi → Arduino    DRIVE 180 5000
50ms     Pi → Arduino    PI_STOP                              ← overrides immediately
50ms     Arduino → Pi    [ACK] PI_STOP
50ms     Arduino → Pi    [TELEM] STATE=PI_STOPPED
         (car frozen after only 50ms of movement)

100ms    Pi → Arduino    DRIVE 150 3000                       ← sent while stopped
100ms    Arduino → Pi    [ACK] BLOCKED (PI_STOPPED)           ← rejected!

200ms    Pi → Arduino    PI_GO
200ms    Arduino → Pi    [ACK] PI_GO
200ms    Arduino → Pi    [TELEM] STATE=IDLE SPEED=0

300ms    Pi → Arduino    DRIVE 150 3000                       ← NOW it works
300ms    Arduino → Pi    [ACK] DRIVE 150 3000

  ── Case D: Multiple PI_AVOID (last one wins) ──

0ms      Pi → Arduino    PI_AVOID LEFT
0ms      Arduino → Pi    [ACK] PI_AVOID LEFT

100ms    Pi → Arduino    PI_AVOID RIGHT                       ← overwrites LEFT
100ms    Arduino → Pi    [ACK] PI_AVOID RIGHT

         (if ultrasonic triggers now, car turns RIGHT — last hint wins)
```

## Key Takeaway
- **No command queue** — each new driving command replaces the previous one immediately
- **`PI_STOP` is instant** — cuts through anything, even commands sent in the same serial buffer
- **Commands sent during `PI_STOPPED` are rejected** with a `BLOCKED` acknowledgment
- **`PI_AVOID` is overwritable** — the last hint sent before an ultrasonic trigger is the one that gets used
- The Pi should wait for `[ACK]` responses before sending the next command if ordering matters (e.g., the pothole navigation sequence in Scenario 5)
