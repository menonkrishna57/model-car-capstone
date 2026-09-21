# Scenario 1: Clear Path — Normal Forward Driving

## Situation
The car is driving forward on an open path. No walls, no obstacles, no potholes. Pi is connected but has nothing to report.

## Initial State
`STATE_IDLE` — motors off, awaiting commands.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Pi → Arduino    DRIVE 180 5000
0ms      Arduino → Pi    [ACK] DRIVE 180 5000
0ms      Arduino → Pi    [TELEM] STATE=DRIVING SPEED=180
100ms    Arduino → Pi    [TELEM] US LEFT=85.2 RIGHT=92.1
200ms    Arduino → Pi    [TELEM] US LEFT=88.0 RIGHT=90.5
300ms    Arduino → Pi    [TELEM] US LEFT=84.7 RIGHT=91.8
  ...    (ultrasonic telemetry continues every 100ms)
5000ms   Arduino → Pi    [TELEM] DONE DRIVE
5000ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0
```

## Motor Behavior

| Phase | Duration | Left Motors | Right Motors |
|---|---|---|---|
| Forward drive | 5000ms | PWM 180 forward | PWM 180 forward |
| Stop (timer expired) | — | 0 | 0 |

## Final State
`STATE_IDLE` — motors off. Car stopped naturally after the timed duration.

## Key Takeaway
This is the simplest case. Ultrasonic readings are broadcast as telemetry but never trigger avoidance because all readings are well above 20cm.
