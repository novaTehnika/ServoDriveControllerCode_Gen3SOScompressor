# Homing Sequence Guide

## Gen3 SOS Compressor Servo Controller
### Detailed Homing Procedures

---

## 1. Overview

The system requires two types of homing to establish accurate position reference:

| Mode | Name | Purpose | When Required |
|------|------|---------|---------------|
| 110 | Home to Limit | Establish absolute position using limit switch | Encoder battery failure or position data corruption |
| 111 | Home to EOT | Calibrate end-of-travel position | Every power cycle |
| 101 | Go Home | Move to home position OR redirect to Mode 110 | User convenience |

### Homing Requirement Flags

| Flag | Set When | Cleared By |
|------|----------|------------|
| `flagAbsHomeRequired` | Encoder invalid on startup | Completing Mode 110 |
| `flagEOTHomeRequired` | Every power cycle | Completing Mode 111 |

---

## 2. Mode 110: Home to Limit Switch

### Purpose
Establish the coordinate system zero reference using the physical home limit switch (DI5). This is required when the absolute encoder cannot provide valid position data.

> **Note on Implementation**
>
> The sequence of steps described below is now encapsulated within the `FB_HomeLimit` function block. The state names (e.g., `ST_HOME_LIM_APPROACH`) are deprecated in `PRG_Main` but are preserved here to accurately describe the internal logic of the function block.

### Sequence Steps

```
Step 1: APPROACH
+------------------------------------------+
| State: ST_HOME_LIM_APPROACH              |
| Action: MC_MoveVelocity (negative dir)   |
| Velocity: cfgHomeLimApproachVelocity     |
|           (default: -20 mm/s)            |
| Exit: LimitHomeActive = TRUE             |
+------------------------------------------+
        |
        v
Step 2: DETECT
+------------------------------------------+
| State: ST_HOME_LIM_DETECT                |
| Action: MC_Halt (controlled stop)        |
| Verify: LimitHomeActive still TRUE       |
| Exit: Motion stopped                     |
+------------------------------------------+
        |
        v
Step 3: BACKOFF
+------------------------------------------+
| State: ST_HOME_LIM_BACKOFF               |
| Action: MC_MoveVelocity (positive dir)   |
| Velocity: cfgHomeLimBackoffVelocity      |
|           (default: +5 mm/s)             |
| Exit: LimitHomeActive = FALSE            |
+------------------------------------------+
        |
        v
Step 4: SET REFERENCE
+------------------------------------------+
| State: ST_HOME_LIM_SETREF                |
| Action: MC_SetPosition                   |
| Position: cfgHomeLimSetPosition          |
|           (default: 0.0 mm)              |
| Clear: flagAbsHomeRequired = FALSE       |
| Exit: MC_SetPosition.Done                |
+------------------------------------------+
        |
        v
Step 5: COMPLETE
+------------------------------------------+
| State: ST_HOME_COMPLETE                  |
| Action: Hold position                    |
| Output: doHomingComplete = TRUE          |
| Exit: New mode commanded                 |
+------------------------------------------+
```

### Timing Expectations

| Phase | Typical Duration | Maximum |
|-------|------------------|---------|
| Approach | 5-15 seconds | 30 seconds |
| Detect | 100 ms | 500 ms |
| Backoff | 1-2 seconds | 5 seconds |
| Set Reference | 50 ms | 200 ms |
| **Total** | **7-18 seconds** | **36 seconds** |

### Abort Handling

If `diMotionEnable` goes LOW during homing:
1. MC_Halt executed immediately
2. Homing sequence aborted
3. Transition to ST_HOLD_POSITION
4. Homing must be restarted from beginning

### Master Coordination

```
MASTER:
1. Command mode 110 (DI0=0, DI1=1, DI2=1)
2. Set diMotionEnable = HIGH
3. Wait for confirmation (DO0=0, DO1=1, DO2=1)
4. Monitor progress:
   - doInMotion = TRUE during approach/backoff
   - doInMotion = FALSE during detect/setref
5. Wait for doHomingComplete = TRUE
6. Homing success - can now command other modes
```

---

## 3. Mode 111: Home to End-of-Travel

### Purpose
Calibrate the cylinder end-of-travel position by detecting mechanical stall. This ensures the position reference accounts for any mechanical variations or wear.

> **Note on Implementation**
>
> The sequence of steps described below is now encapsulated within the `FB_HomeEOT` function block. The state names (e.g., `ST_HOME_EOT_FAST`) are deprecated in `PRG_Main` but are preserved here to accurately describe the internal logic of the function block.

### Sequence Steps

```
Step 1: FAST APPROACH
+------------------------------------------+
| State: ST_HOME_EOT_FAST                  |
| Action: MC_MoveVelocity (positive dir)   |
| Velocity: cfgHomeEOTFastVelocity         |
|           (default: +50 mm/s)            |
| Exit: Position near expected EOT         |
|       (cfgHomeEOTSlowStartPos)           |
+------------------------------------------+
        |
        v
Step 2: SLOW APPROACH
+------------------------------------------+
| State: ST_HOME_EOT_SLOW                  |
| Action: Y_DirectControl (velocity mode)  |
| Velocity: cfgHomeEOTSlowVelocity         |
|           (default: +10 mm/s)            |
| Torque Limit: cfgHomeEOTTorqueLimit      |
|               (default: 30%)             |
| Exit: Velocity drops below threshold     |
+------------------------------------------+
        |
        v
Step 3: STALL DETECT
+------------------------------------------+
| State: ST_HOME_EOT_DETECT                |
| Condition 1: |velocity| < 0.5 mm/s       |
| Condition 2: |torque| >= 90% of limit    |
| Duration: 200 ms continuous              |
| Action: Confirm mechanical stop          |
| Exit: Stall confirmed                    |
+------------------------------------------+
        |
        v
Step 4: SET REFERENCE
+------------------------------------------+
| State: ST_HOME_EOT_SETREF                |
| Action: MC_SetPosition                   |
| Position: cfgHomeEOTSetPosition          |
|           (default: 305.0 mm)            |
| Clear: flagEOTHomeRequired = FALSE       |
| Exit: MC_SetPosition.Done                |
+------------------------------------------+
        |
        v
Step 5: COMPLETE
+------------------------------------------+
| State: ST_HOME_COMPLETE                  |
| Action: Hold position                    |
| Output: doHomingComplete = TRUE          |
| Exit: New mode commanded                 |
+------------------------------------------+
```

### Stall Detection Algorithm

```
STALL DETECTION CRITERIA:
  - Actual velocity magnitude < cfgHomeEOTStallVelocity (0.5 mm/s)
  - Actual torque magnitude >= cfgHomeEOTStallTorquePercent (90%) of limit
  - Both conditions met for cfgHomeEOTStallTime (200 ms)

PSEUDOCODE:
IF ABS(actual_velocity) < 0.5 AND
   ABS(actual_torque) >= 0.9 * torque_limit THEN
    IF stall_timer >= 200ms THEN
        stall_confirmed = TRUE
    ELSE
        stall_timer = stall_timer + scan_time
    END_IF
ELSE
    stall_timer = 0
END_IF
```

### Timing Expectations

| Phase | Typical Duration | Maximum |
|-------|------------------|---------|
| Fast Approach | 3-5 seconds | 10 seconds |
| Slow Approach | 1-3 seconds | 10 seconds |
| Stall Detect | 200-500 ms | 2 seconds |
| Set Reference | 50 ms | 200 ms |
| **Total** | **5-9 seconds** | **22 seconds** |

### Safety Considerations

- Torque limit prevents motor/mechanical damage during stall
- Slow approach velocity reduces impact energy
- Stall confirmation time prevents false triggers
- If stall not detected within timeout, fault is raised

### Master Coordination

```
MASTER:
1. Ensure Mode 110 completed (if encoder was invalid)
2. Command mode 111 (DI0=1, DI1=1, DI2=1)
3. Set diMotionEnable = HIGH
4. Wait for confirmation (DO0=1, DO1=1, DO2=1)
5. Monitor progress:
   - doInMotion = TRUE during approach phases
   - doInMotion = FALSE when stall detected
6. Wait for doHomingComplete = TRUE
7. Homing complete - full position calibration done
```

---

## 4. Mode 101: Go Home

### Purpose
Move to the home position (at the home limit switch location). Provides convenience for returning to known position, with automatic handling of homing requirements.

### Behavior Decision Tree

```
GO HOME REQUESTED
        |
        v
+-------------------+
| Check             |
| flagAbsHomeRequired|
+--------+----------+
         |
    +----+----+
    |         |
   TRUE     FALSE
    |         |
    v         v
+-------+  +------------------+
|Redirect|  | Execute         |
|to      |  | MC_MoveAbsolute |
|Mode 110|  | to HomePosition |
+-------+  +------------------+
    |              |
    v              v
 Run Mode 110   Hold at home
 sequence       until mode change
```

### When Redirect Occurs

If `flagAbsHomeRequired = TRUE`:
1. FB_GoHome sets `RedirectToHoming = TRUE`
2. PRG_Main transitions to ST_HOME_LIMIT (Mode 110)
3. Mode 110 sequence executes
4. After completion, system is at home position
5. `flagAbsHomeRequired` is cleared

### Normal Go Home Execution

If `flagAbsHomeRequired = FALSE`:
1. Execute MC_MoveAbsolute to `cfgGoHomePosition` (default: 0.0 mm)
2. Velocity: `cfgGoHomeVelocity` (default: 50 mm/s)
3. Hold at position until mode change

### Important Notes

- EOT homing (`flagEOTHomeRequired`) does NOT block Go Home
- Go Home can execute even if EOT homing not done
- Only encoder validity (`flagAbsHomeRequired`) causes redirect
- After Go Home, position is at home switch location

### Master Coordination

```
MASTER:
1. Command mode 101 (DI0=1, DI1=0, DI2=1)
2. Set diMotionEnable = HIGH
3. Wait for confirmation
4. IF confirmation changes to 110:
   - Redirect occurred, Mode 110 running
   - Wait for that sequence to complete
   ELSE:
   - Direct move to home executing
   - Wait for doInMotion = FALSE
5. Axis is now at home position
```

---

## 5. Homing Configuration Parameters

### Mode 110 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| cfgHomeLimApproachVelocity | -20.0 | mm/s | Approach velocity (negative = toward switch) |
| cfgHomeLimBackoffVelocity | +5.0 | mm/s | Backoff velocity (positive = away from switch) |
| cfgHomeLimSetPosition | 0.0 | mm | Position value after homing |
| cfgHomeLimTimeout | 30.0 | s | Maximum time for homing |

### Mode 111 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| cfgHomeEOTFastVelocity | +50.0 | mm/s | Fast approach velocity |
| cfgHomeEOTSlowStartPos | 280.0 | mm | Position to start slow approach |
| cfgHomeEOTSlowVelocity | +10.0 | mm/s | Slow approach velocity |
| cfgHomeEOTTorqueLimit | 30.0 | % | Torque limit during stall |
| cfgHomeEOTStallVelocity | 0.5 | mm/s | Velocity threshold for stall |
| cfgHomeEOTStallTorquePercent | 90.0 | % | Torque threshold (of limit) |
| cfgHomeEOTStallTime | 200 | ms | Stall confirmation time |
| cfgHomeEOTSetPosition | 305.0 | mm | Position value at EOT |
| cfgHomeEOTTimeout | 30.0 | s | Maximum time for homing |

### Mode 101 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| cfgGoHomePosition | 0.0 | mm | Target home position |
| cfgGoHomeVelocity | 50.0 | mm/s | Movement velocity |

---

## 6. Typical Startup Homing Sequence

### After Normal Power Cycle

```
POWER ON
    |
    v
Encoder Check --> Valid
    |
    v
flagAbsHomeRequired = FALSE
flagEOTHomeRequired = TRUE (always on power-up)
    |
    v
MASTER: Command Mode 111 (Home to EOT)
    |
    v
EOT homing executes
    |
    v
flagEOTHomeRequired = FALSE
doHomingComplete = TRUE
    |
    v
Ready for operational modes
```

### After Encoder Battery Failure

```
POWER ON
    |
    v
Encoder Check --> INVALID (battery fail)
    |
    v
flagAbsHomeRequired = TRUE
flagEOTHomeRequired = TRUE
    |
    v
MASTER: Command Mode 110 (Home to Limit)
    |
    v
Limit homing executes
    |
    v
flagAbsHomeRequired = FALSE
    |
    v
MASTER: Command Mode 111 (Home to EOT)
    |
    v
EOT homing executes
    |
    v
flagEOTHomeRequired = FALSE
doHomingComplete = TRUE
    |
    v
Ready for operational modes
```

---

## 7. Troubleshooting

### Homing Timeout

**Symptom**: Homing doesn't complete within expected time

**Possible Causes**:
- Limit switch not triggering (wiring issue)
- Mechanical obstruction
- Velocity too slow
- Position wrong (switch in unexpected location)

**Resolution**:
1. Check limit switch wiring and function
2. Verify mechanical path is clear
3. Check homing velocity parameters
4. Manually inspect switch position

### Stall Not Detected (Mode 111)

**Symptom**: EOT homing runs but never confirms stall

**Possible Causes**:
- Torque limit too high (motor doesn't reach threshold)
- Stall velocity threshold too low
- Mechanical friction too high
- Not actually reaching EOT

**Resolution**:
1. Reduce torque limit parameter
2. Increase stall velocity threshold slightly
3. Check mechanical friction
4. Verify expected EOT position

### Frequent Homing Required

**Symptom**: `flagAbsHomeRequired` set frequently

**Possible Causes**:
- Encoder battery failing
- Electrical noise on encoder signals
- Loose encoder cable

**Resolution**:
1. Check encoder battery voltage (drive parameter)
2. Replace encoder battery proactively
3. Verify encoder cable shielding and routing

### Position Drift After Homing

**Symptom**: Position accuracy degrades over time

**Possible Causes**:
- EOT homing not performed (every power cycle)
- Mechanical wear changing EOT position
- Thermal expansion effects

**Resolution**:
1. Always complete Mode 111 after power cycle
2. Periodically re-run EOT homing during long operations
3. Consider temperature compensation if significant

---

## 8. Quick Reference

### Homing Mode Commands

| Mode | Binary | Action |
|------|--------|--------|
| 101 | 101 | Go Home (or redirect to 110) |
| 110 | 110 | Home to Limit Switch |
| 111 | 111 | Home to End-of-Travel |

### Flag Clearance

| Flag | Cleared By |
|------|------------|
| flagAbsHomeRequired | Mode 110 complete |
| flagEOTHomeRequired | Mode 111 complete |

### Required Sequence

1. If encoder invalid: Mode 110 first
2. Every power cycle: Mode 111
3. Then operational modes available
