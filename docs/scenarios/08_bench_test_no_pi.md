# Scenario 8: Bench Test — No Pi Connected (Serial Monitor Only)

## Situation
You're testing with just the Arduino Uno and a USB cable to your laptop. No Pi is connected. You're using the Arduino IDE Serial Monitor to send commands and observe telemetry. This is how you'll test when the Uno is available but the Pi isn't set up yet.

## Initial State
Power on → Arduino runs `setup()` → `STATE_IDLE`.

## Serial Monitor Setup
- **Baud rate**: 9600
- **Line ending**: Newline (`\n`)

## Serial Message Flow

```
TIME     DIRECTION          MESSAGE
─────────────────────────────────────────────────────
         Arduino → Monitor  [TELEM] READY Uno-ATmega328P

  ── Test 1: Basic driving ──

You type: DRIVE 150 3000
         Arduino → Monitor  [ACK] DRIVE 150 3000
         Arduino → Monitor  [TELEM] STATE=DRIVING SPEED=150
         Arduino → Monitor  [TELEM] US LEFT=100.0 RIGHT=95.0
         Arduino → Monitor  [TELEM] US LEFT=98.5 RIGHT=97.2
         ...
3s later Arduino → Monitor  [TELEM] DONE DRIVE
         Arduino → Monitor  [TELEM] STATE=IDLE SPEED=0

  ── Test 2: Manual stop ──

You type: DRIVE 200 10000
         Arduino → Monitor  [ACK] DRIVE 200 10000
         (car moves)
You type: STOP
         Arduino → Monitor  [ACK] STOP
         Arduino → Monitor  [TELEM] STATE=IDLE SPEED=0

  ── Test 3: Ultrasonic avoidance ──

You type: DRIVE 150 10000
         (car moves)
         (you place your hand ~10cm from the left sensor)
         Arduino → Monitor  [TELEM] US LEFT=10.2 RIGHT=88.0
         Arduino → Monitor  [TELEM] AVOID RIGHT
         (car reverses, turns right, resumes)
         Arduino → Monitor  [TELEM] DONE AVOID

  ── Test 4: Simulate Pi commands ──

You type: PI_STOP
         Arduino → Monitor  [ACK] PI_STOP
         Arduino → Monitor  [TELEM] STATE=PI_STOPPED
         (car frozen — try sending DRIVE, it won't work)
You type: DRIVE 150 2000
         Arduino → Monitor  [ACK] BLOCKED (PI_STOPPED)
You type: PI_GO
         Arduino → Monitor  [ACK] PI_GO
         Arduino → Monitor  [TELEM] STATE=IDLE SPEED=0
         (car unfrozen, DRIVE works again)

  ── Test 5: Pi avoid hint ──

You type: PI_AVOID LEFT
         Arduino → Monitor  [ACK] PI_AVOID LEFT
You type: DRIVE 150 10000
         (place hand near RIGHT sensor)
         Arduino → Monitor  [TELEM] US LEFT=85.0 RIGHT=8.0
         Arduino → Monitor  [TELEM] AVOID LEFT
         (car turns LEFT even though obstacle is on RIGHT — hint override!)

  ── Test 6: Reverse and turn ──

You type: REV 150 2000
         Arduino → Monitor  [ACK] REV 150 2000
         (car reverses)
You type: TURN 90 150
         Arduino → Monitor  [ACK] TURN 90 150
         (car skid-turns right for ~900ms)

  ── Test 7: Ping ──

You type: PING
         Arduino → Monitor  [ACK] PONG
```

## Final State
Depends on last command sent. The car is fully controllable from Serial Monitor.

## Key Takeaway
- **Everything works without the Pi** — Serial Monitor acts as a substitute
- You can simulate `PI_STOP`, `PI_GO`, `PI_AVOID` from Serial Monitor to test the full priority system
- This is useful for:
  - Calibrating avoidance timing on the real chassis
  - Testing ultrasonic sensor placement and readings
  - Verifying motor direction and speed
  - Demonstrating the system to teammates
