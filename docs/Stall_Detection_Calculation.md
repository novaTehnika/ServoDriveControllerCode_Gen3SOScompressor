# Stall Detection Calculation for EOT Homing (Mode 111)

## Purpose
This document clarifies the torque threshold calculation used for stall detection during End-of-Travel (EOT) homing in Mode 111.

---

## Specification Reference

**From devPlan_V03.md (line 433)**:
> "Stall detection: Velocity + Torque threshold (velocity < 0.5mm/s AND torque >= 90% threshold for 200ms)"

---

## Implementation Details

### Location
- **PRG_Main.st**: ST_HOME_EOT_SLOW (lines 880-881) and ST_HOME_EOT_DETECT (lines 922-923)

### Code
```st
(* Stall detection condition *)
IF (ABS(sysActualVelocity) < 0.5) AND
   (ABS(sysActualTorque) >= cfgHomeEOTTorqueThresh * 0.9) THEN
    (* Stall detected - proceed to confirmation *)
END_IF
```

---

## Parameter Values (from GVL_Config.st)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `cfgHomeEOTTorqueThresh` | 50.0 | Torque threshold in % of rated |
| `cfgStallDetectTime` | 200 | Confirmation time in ms |
| `cfgHomeEOTSlowVel` | 5.0 | Commanded slow approach velocity (mm/s) |
| `cfgTorqueHomingLimit` | 60.0 | Torque limit during slow approach (%) |

---

## Calculation Breakdown

### Step 1: Torque Limit Applied During Slow Approach
During ST_HOME_EOT_SLOW, Y_DirectControl limits torque to:
- **60%** of rated torque (`cfgTorqueHomingLimit`)

### Step 2: Stall Detection Threshold
The stall is detected when actual torque reaches:
- **90% of 50%** = **45%** of rated torque

### Step 3: Detection Sequence
```
1. Axis approaches EOT at 5 mm/s with 60% torque limit
2. Contact with mechanical stop
3. Velocity drops as resistance increases
4. When: velocity < 0.5 mm/s AND torque >= 45%
   → Stall condition detected
5. Condition must persist for 200ms
   → Stall confirmed
6. Position reference set at current location
```

---

## Interpretation Clarification

The specification phrase "torque >= 90% threshold" could be interpreted two ways:

### Interpretation A (Implemented)
"90% of the configured torque threshold"
- cfgHomeEOTTorqueThresh = 50%
- 90% of 50% = **45% of rated torque**

### Interpretation B (Not Used)
"90% of the torque limit being applied"
- cfgTorqueHomingLimit = 60%
- 90% of 60% = **54% of rated torque**

### Rationale for Interpretation A
1. **Earlier Detection**: Lower threshold (45%) detects stall sooner, reducing mechanical stress
2. **Safety Margin**: Stall detected before torque limit is fully reached
3. **Configurable**: `cfgHomeEOTTorqueThresh` is specifically named for this purpose
4. **Separate Concerns**: Detection threshold is independent of torque limiting

---

## Stall Detection State Diagram

```
ST_HOME_EOT_SLOW
    |
    | Commanding: 5 mm/s with 60% torque limit
    |
    v
[Contact with EOT]
    |
    | Velocity decreasing, torque increasing
    |
    v
[Velocity < 0.5 mm/s AND Torque >= 45%]
    |
    | Stall condition met
    |
    v
ST_HOME_EOT_DETECT
    |
    | Start 200ms timer
    |
    v
[Condition persists for 200ms?]
    |
   / \
  NO  YES
  |    |
  v    v
RETRY  ST_HOME_EOT_SETREF
       (Homing complete)
```

---

## Safety Considerations

### Why Not Use Higher Thresholds?

1. **Mechanical Protection**: Lower torque threshold reduces stress on leadscrew and actuator
2. **Motor Protection**: Prevents sustained high-current operation against stall
3. **Reliable Detection**: Clear velocity drop with moderate torque indicates solid contact

### Why 200ms Confirmation?

1. **Debouncing**: Filters momentary velocity dips or torque spikes
2. **Solid Contact**: Ensures actuator has fully seated against EOT
3. **Not Too Long**: Limits sustained stall current duration

---

## Tuning Guidelines

| Parameter | Effect of Increasing | Effect of Decreasing |
|-----------|---------------------|---------------------|
| `cfgHomeEOTTorqueThresh` | Later detection, more force | Earlier detection, less force |
| `cfgStallDetectTime` | More reliable but slower | Faster but risk of false triggers |
| `cfgHomeEOTSlowVel` | Faster approach, harder impact | Slower approach, softer contact |
| `cfgTorqueHomingLimit` | More force available | Gentler but may not reach EOT |

### Recommended Starting Values (Current Defaults)
- Torque threshold: 50% (detects at 45%)
- Confirmation time: 200ms
- Slow velocity: 5 mm/s
- Torque limit: 60%

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-29 | Initial documentation |
