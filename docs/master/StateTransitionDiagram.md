# State Transition Diagram

## Gen3 SOS Compressor Servo Controller
### Visual State Machine Reference

---

## 1. High-Level State Overview

```
                           POWER ON
                               |
                               v
+------------------------------------------------------------------+
|                    INITIALIZATION PHASE                           |
|  +----------+     +--------------+     +----------------+         |
|  | ST_INIT  | --> | ST_ENCODER   | --> | ST_REQUIRE     |         |
|  |          |     | _CHECK       |     | _ABS_HOME      |         |
|  +----------+     +--------------+     +----------------+         |
|                          |                    |                   |
|                     (Valid)              (Invalid)                |
|                          |                    |                   |
+--------------------------|--------------------|---------+---------+
                           |                    |         |
                           v                    v         |
                    +----------+                |         |
                    | ST_IDLE  |<---------------+         |
                    +----------+                          |
                           |                              |
            (Mode != 000 + diMotionEnable)                |
                           |                              |
            +--------------+--------------+               |
            |                             |               |
     (Homing Modes              (Operational Modes        |
      101,110,111)               001,010,011,100)         |
            |                             |               |
            |              Check homing requirements      |
            |                     /     \                 |
            |                  (MET)   (NOT MET)          |
            |                    |         |              |
            |                    |         v              |
            |                    |   +-----------+        |
            |                    |   | ST_FAULT  |--------+
            |                    |   | (HOMING   |
            |                    |   |  _REQ)    |
            |                    |   +-----------+
            |                    |
            +----------+---------+
                       |
                       v
              +----------------+
              | ST_DRIVE_ENABLE|
              +----------------+
                       |
                       v
              +----------------+
              | ST_BRAKE       |
              | _RELEASE       |
              +----------------+
                       |
         +------+------+------+------+------+------+------+
         |      |      |      |      |      |      |      |
         v      v      v      v      v      v      v      v
       001    010    011    100    101    110    111
       BRAKE  POS    VEL    TRQ    GO     HOME   HOME
       HOLD   CTRL   CTRL   CTRL   HOME   LIMIT  EOT
```

---

## 2. Operational State Flow

```
                    OPERATIONAL STATES
    +------------------------------------------------+
    |                                                |
    |   +----------+    +----------+    +----------+ |
    |   | POSITION |    | VELOCITY |    | TORQUE   | |
    |   | CTRL     |    | CTRL     |    | CTRL     | |
    |   | (010)    |    | (011)    |    | (100)    | |
    |   +----+-----+    +----+-----+    +----+-----+ |
    |        |              |              |         |
    |        +------+-------+------+-------+         |
    |               |              |                 |
    |      (diMotionEnable   (Mode change           |
    |       = LOW)            or fault)             |
    |               |              |                 |
    |               v              v                 |
    |        +-------------+  +-----------+          |
    |        | ST_HOLD     |  | ST_FAULT  |          |
    |        | _POSITION   |  |           |          |
    |        +------+------+  +-----------+          |
    |               |                                |
    |      (New mode handshake)                      |
    |               |                                |
    |        +------v------+                         |
    |        | New Mode    |                         |
    |        | (via brake  |                         |
    |        |  release)   |                         |
    |        +-------------+                         |
    +------------------------------------------------+
```

---

## 3. Homing State Sequences

### Mode 110: Home to Limit Switch

```
+---------------+
| ST_HOME_LIMIT |  Entry point from mode command
+-------+-------+
        |
        v
+-------------------+
| ST_HOME_LIM       |  MC_MoveVelocity (negative direction)
| _APPROACH         |  Moving toward home limit switch
+-------+-----------+
        |
        | (LimitHomeActive = TRUE)
        v
+-------------------+
| ST_HOME_LIM       |  MC_Halt
| _DETECT           |  Confirm switch triggered
+-------+-----------+
        |
        v
+-------------------+
| ST_HOME_LIM       |  MC_MoveVelocity (positive, slow)
| _BACKOFF          |  Back away from switch
+-------+-----------+
        |
        | (LimitHomeActive = FALSE)
        v
+-------------------+
| ST_HOME_LIM       |  MC_SetPosition
| _SETREF           |  Set position reference
+-------+-----------+  Clear flagAbsHomeRequired
        |
        v
+---------------+
| ST_HOME       |  Wait for mode change
| _COMPLETE     |
+---------------+
```

### Mode 111: Home to End-of-Travel

```
+---------------+
| ST_HOME_EOT   |  Entry point from mode command
+-------+-------+
        |
        v
+-------------------+
| ST_HOME_EOT       |  MC_MoveVelocity (positive, fast)
| _FAST             |  Fast jog toward EOT region
+-------+-----------+
        |
        | (Near expected EOT region)
        v
+-------------------+
| ST_HOME_EOT       |  Y_DirectControl (velocity + torque limit)
| _SLOW             |  Slow approach with torque limiting
+-------+-----------+
        |
        | (Stall detected: vel < 0.5mm/s AND torque >= 90%)
        v
+-------------------+
| ST_HOME_EOT       |  Confirm stall for 200ms
| _DETECT           |  Validate mechanical stop reached
+-------+-----------+
        |
        v
+-------------------+
| ST_HOME_EOT       |  MC_SetPosition
| _SETREF           |  Set position reference
+-------+-----------+  Clear flagEOTHomeRequired
        |
        v
+---------------+
| ST_HOME       |  Wait for mode change
| _COMPLETE     |
+---------------+
```

### Mode 101: Go Home

```
+---------------+
| ST_GO_HOME    |  Entry point from mode command
+-------+-------+
        |
        | Check flagAbsHomeRequired
        |
   +----+----+
   |         |
  TRUE     FALSE
   |         |
   v         v
+-------+  +-------------------+
|Redirect|  | MC_MoveAbsolute  |
|to      |  | to HomePosition  |
|Mode 110|  +--------+---------+
+-------+           |
   |                | (At position)
   v                v
+-----------+  +---------------+
|ST_HOME    |  | Hold position |
|_LIMIT     |  | until mode    |
|(sub-seq)  |  | change        |
+-----------+  +---------------+
```

---

## 4. Fault Handling Flow

```
        ANY STATE
            |
            | (Fault condition detected)
            v
    +---------------+
    | ST_FAULT      |
    |               |
    | - doFaultActive = HIGH
    | - Output fault code on DO0-DO2
    | - Motion stopped
    | - Brake engaged
    +-------+-------+
            |
            | (diFaultReset rising edge
            |  with valid mirrored code
            |  and diMotionEnable = LOW)
            v
    +---------------+
    | ST_FAULT      |
    | _RECOVERY     |
    |               |
    | - Validate mirrored code
    | - Clear fault latch
    +-------+-------+
            |
            v
    +---------------+
    | ST_BRAKE      |
    | _ENGAGE       |
    +-------+-------+
            |
            v
    +---------------+
    | ST_DRIVE      |
    | _DISABLE      |
    +-------+-------+
            |
            v
    +---------------+
    | ST_IDLE       |
    +---------------+
```

---

## 5. Valid Mode Transitions

### From IDLE (Mode 000)

| To Mode | Condition | Path |
|---------|-----------|------|
| 001 (Brake Hold) | Homing met OR homing modes | DRIVE_ENABLE → BRAKE_RELEASE → BRAKE_HOLD |
| 010 (Position) | Homing met | DRIVE_ENABLE → BRAKE_RELEASE → POSITION_CTRL |
| 011 (Velocity) | Homing met | DRIVE_ENABLE → BRAKE_RELEASE → VELOCITY_CTRL |
| 100 (Torque) | Homing met | DRIVE_ENABLE → BRAKE_RELEASE → TORQUE_CTRL |
| 101 (Go Home) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → GO_HOME |
| 110 (Home Limit) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → HOME_LIMIT |
| 111 (Home EOT) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → HOME_EOT |

### From Operational Modes

| From | To | Condition | Path |
|------|----|-----------| -----|
| Any Operational | Different Mode | diMotionEnable LOW→HIGH | HOLD_POSITION → DRIVE_ENABLE → New Mode |
| Any Operational | IDLE (000) | diMotionEnable LOW | HOLD_POSITION → timeout → FAULT |
| Any Operational | FAULT | Fault detected | Direct transition |

### Homing Mode Exits

| From | To | Condition | Path |
|------|----|-----------| -----|
| HOME_COMPLETE | Any Mode | New mode command | Handshake → DRIVE_ENABLE → New Mode |
| Any Homing State | FAULT | Abort or timeout | Direct transition |

---

## 6. State Enumeration Reference

```
STATE                      VALUE    DESCRIPTION
---------------------------------------------------------
ST_INIT                    0        Power-on initialization
ST_ENCODER_CHECK           1        Validate absolute encoder
ST_REQUIRE_ABS_HOME        2        Set homing required flag
ST_IDLE                    10       Drive OFF, awaiting commands
ST_DRIVE_ENABLE            20       Powering on drive
ST_BRAKE_RELEASE           21       Releasing brake
ST_BRAKE_HOLD              30       Mode 001: Drive ON, brake ON
ST_POSITION_CTRL           31       Mode 010: Position control
ST_VELOCITY_CTRL           32       Mode 011: Velocity control
ST_TORQUE_CTRL             33       Mode 100: Torque control
ST_GO_HOME                 34       Mode 101: Go home sequence
ST_HOME_LIMIT              40       Mode 110: Entry
ST_HOME_LIM_APPROACH       41       Moving to limit
ST_HOME_LIM_DETECT         42       Detecting limit
ST_HOME_LIM_BACKOFF        43       Backing off
ST_HOME_LIM_SETREF         44       Setting reference
ST_HOME_EOT                50       Mode 111: Entry
ST_HOME_EOT_FAST           51       Fast approach
ST_HOME_EOT_SLOW           52       Slow approach
ST_HOME_EOT_DETECT         53       Stall detection
ST_HOME_EOT_SETREF         54       Setting reference
ST_HOME_COMPLETE           60       Homing done
ST_HOLD_POSITION           70       Controlled stop, await mode
ST_BRAKE_ENGAGE            80       Engaging brake
ST_DRIVE_DISABLE           81       Powering off drive
ST_FAULT                   90       Fault active
ST_FAULT_RECOVERY          91       Processing reset
```

---

## 7. Master State Machine

The master should implement a complementary state machine:

```
MASTER STATES
+----------------------------------------------------------+
|                                                          |
|  +--------+                                              |
|  | INIT   | --> Initialize DAQ, set outputs LOW          |
|  +----+---+                                              |
|       |                                                  |
|       v                                                  |
|  +--------+                                              |
|  | IDLE   | <--+  Monitor doFaultActive                  |
|  +----+---+    |  Mode bits = 000                        |
|       |        |                                         |
|       | (Mode request)                                   |
|       v        |                                         |
|  +----------+  |                                         |
|  | REQUEST  |  |  Set mode bits, raise diMotionEnable    |
|  | _MODE    |  |  Start handshake timer                  |
|  +----+-----+  |                                         |
|       |        |                                         |
|   +---+---+    |                                         |
|   |       |    |                                         |
| (Match) (Timeout)                                        |
|   |       |    |                                         |
|   v       v    |                                         |
|  +------+ +----+---+                                     |
|  | OPER | | TIMEOUT|  Handle timeout (wait for fault)    |
|  | ATING| | _WAIT  |                                     |
|  +--+---+ +--------+                                     |
|     |          |                                         |
|     | (Mode change)                                      |
|     v          |                                         |
|  +--------+    |                                         |
|  | STOP   |----+  Drop diMotionEnable, wait for halt     |
|  | _MOTION|                                              |
|  +--------+                                              |
|                                                          |
|  +--------+                                              |
|  | FAULT  | <-- When doFaultActive goes HIGH             |
|  | _HANDLE|     Read code, mirror, reset                 |
|  +----+---+                                              |
|       |                                                  |
|       | (Fault cleared)                                  |
|       v                                                  |
|  +--------+                                              |
|  | IDLE   |                                              |
|  +--------+                                              |
|                                                          |
+----------------------------------------------------------+
```

---

## 8. Timing Considerations

### Critical Timing Points

| Transition | Timing | Notes |
|------------|--------|-------|
| Mode request → Confirm | < 500 ms | Handshake timeout |
| diMotionEnable LOW → doInMotion LOW | Variable | Deceleration time |
| Fault detect → Fault code stable | < 10 ms | Wait before reading |
| diFaultReset edge → doFaultActive LOW | < 1000 ms | Reset timeout |
| Brake engage → Brake hold | 200 ms | Mechanical delay |
| Brake release → Motion allowed | 100 ms | Mechanical delay |

### State Transition Timing

```
Time ------>

diMotionEnable  _____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________
                     ^                        ^
                     |                        |
                  Request                   Stop
                  mode                      motion

Slave State     IDLE | ENABLE | RELEASE | OPERATING | HOLD | ENGAGE | IDLE
                     |<-100ms->|<-100ms->|           |      |<-200ms>|

Mode Confirm    000__|________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_____|__000___
                     ^        ^                             ^
                     |        |                             |
                  Request  Confirmed                     Cleared
```

---

## 9. Decision Points Summary

### At ST_IDLE
- Check: `diMotionEnable` rising edge?
- Check: Mode bits != 000?
- Check: If operational mode, are homing requirements met?

### At Operational State
- Check: `diMotionEnable` falling edge → ST_HOLD_POSITION
- Check: Fault condition → ST_FAULT
- Check: Mode bits changed (invalid) → ST_FAULT

### At ST_HOLD_POSITION
- Check: New mode handshake started?
- Check: Handshake timeout → ST_FAULT

### At ST_FAULT
- Check: Valid reset handshake?
- Check: Fault condition persists after reset?
