# Scenario 5: Pothole Ahead — Pi Camera Detection (No Physical Obstacle)

## Situation
The car is driving forward on a clear path. Ultrasonics read clear on both sides (no walls). The Pi 5's camera + YOLO algorithm detects a **pothole** (negative obstacle) directly ahead. The ultrasonic sensors **cannot** detect potholes — this is purely a camera/vision task.

## Initial State
`STATE_DRIVING` — moving forward at PWM 180.

## Why ultrasonics can't help here
HC-SR04P sensors fire sound pulses **horizontally** to the left and right. A pothole is a **hole in the ground** — there's nothing for the sound to bounce off of. Only the camera looking forward and down can see it.

## Serial Message Flow

```
TIME     DIRECTION       MESSAGE
─────────────────────────────────────────────────────
0ms      Arduino → Pi    [TELEM] US LEFT=90.0 RIGHT=88.0     ← all clear
0ms      Arduino → Pi    [TELEM] STATE=DRIVING SPEED=180

         (Pi's YOLO algorithm detects pothole in camera frame)

150ms    Pi → Arduino    PI_STOP
150ms    Arduino → Pi    [ACK] PI_STOP
150ms    Arduino → Pi    [TELEM] STATE=PI_STOPPED

         (Car is now completely frozen. Ultrasonics still polled
          but avoidance CANNOT trigger — Pi latch overrides.)

         (Pi's algorithm analyzes the scene, decides left is clear)

300ms    Pi → Arduino    PI_AVOID LEFT                         ← hint stored
300ms    Arduino → Pi    [ACK] PI_AVOID LEFT

         (Pi releases the latch)

500ms    Pi → Arduino    PI_GO
500ms    Arduino → Pi    [ACK] PI_GO
500ms    Arduino → Pi    [TELEM] STATE=IDLE SPEED=0

         (Pi now navigates the car around the pothole manually)

600ms    Pi → Arduino    TURN -60 150                          ← turn left 60°
600ms    Arduino → Pi    [ACK] TURN -60 150
600ms    Arduino → Pi    [TELEM] STATE=TURNING

1200ms   Arduino → Pi    [TELEM] DONE TURN
1200ms   Arduino → Pi    [TELEM] STATE=IDLE SPEED=0

1300ms   Pi → Arduino    DRIVE 150 2000                        ← drive past pothole
1300ms   Arduino → Pi    [ACK] DRIVE 150 2000
1300ms   Arduino → Pi    [TELEM] STATE=DRIVING SPEED=150

3300ms   Arduino → Pi    [TELEM] DONE DRIVE

3400ms   Pi → Arduino    TURN 60 150                           ← straighten out
3400ms   Arduino → Pi    [ACK] TURN 60 150

4000ms   Arduino → Pi    [TELEM] DONE TURN

4100ms   Pi → Arduino    DRIVE 180 5000                        ← resume normal speed
4100ms   Arduino → Pi    [ACK] DRIVE 180 5000
```

## Motor Behavior

| Phase | Who decides | Duration | Left Motors | Right Motors |
|---|---|---|---|---|
| Driving forward | Pi (DRIVE cmd) | until PI_STOP | PWM 180 fwd | PWM 180 fwd |
| **FROZEN** | Pi (PI_STOP) | ~350ms | **0** | **0** |
| Turn left 60° | Pi (TURN cmd) | ~600ms | PWM 150 backward | PWM 150 forward |
| Drive past pothole | Pi (DRIVE cmd) | 2000ms | PWM 150 forward | PWM 150 forward |
| Turn right 60° | Pi (TURN cmd) | ~600ms | PWM 150 forward | PWM 150 backward |
| Resume driving | Pi (DRIVE cmd) | 5000ms | PWM 180 forward | PWM 180 forward |

## Final State
`STATE_DRIVING` — car has navigated around the pothole and is back on course.

## Key Takeaway
- **Ultrasonics are blind to potholes.** Only the camera can see them.
- The Pi is the **brain** here — it stops the car, plans the route, and sends step-by-step commands.
- The Arduino just executes `TURN` and `DRIVE` commands as instructed.
- If a wall suddenly appears during the Pi's maneuver, the ultrasonic reflex (Tier 2) will interrupt and avoid it — then the Pi can re-plan.
- The `PI_AVOID LEFT` hint sent at 300ms would only be used if an ultrasonic trigger happened — since it didn't, the hint stays stored (cleared on next use or overwritten by a new `PI_AVOID`).
