# Plan: Phase 4-5 Review Implementation Fixes

**Based on**: notes_phase4-5_rev00.md review feedback
**Status**: FINAL

---

## User Decisions (Confirmed)

1. **Offset Application**: Apply in PRG_Main before passing to motion FBs
2. **Fault Brake**: No brake during active fault; engage brake only after timeout to ST_FAULT_IDLE
3. **Piston Exit Guard**: Detect and fault ONLY - remove ALL command correction logic

---

## Summary of Changes

### Change 1: Dual Coordinate System (MAJOR)
**Files**: GVL_System.st, PRG_Main.st

**Problem**: fbSetPosition in ST_HOME_EOT_SETREF overwrites the zero reference from limit switch homing.

**Solution**:
- Add `posEOTOffset` and `posActualEOT` variables to GVL_System.st
- In ST_HOME_EOT_SETREF: DO NOT call fbSetPosition
  - Record actual encoder position at stall: `posActualEOT := sysActualPosition`
  - Calculate offset: `posEOTOffset := cfgHomeEOTSetPosition - posActualEOT`
- Apply offset in PRG_Main:
  - Position commands: `sysCommandedPosition := fbAnalogProc.ScaledPosition - posEOTOffset`
  - Position feedback: `fbPosMapping(ActualPosition := sysActualPosition + posEOTOffset)`

---

### Change 2: Fault State Architecture (MAJOR)
**Files**: E_SystemState.st, PRG_Main.st

**Problem**: ST_FAULT engages brake and times out to ST_IDLE (not designed for fault handling).

**Solution**:
- Add `ST_FAULT_IDLE := 92` to E_SystemState.st
- Modify ST_FAULT:
  - Remove `doBrakeDisengage := FALSE` (no brake during active fault)
  - Change timeout transition: ST_FAULT → ST_FAULT_IDLE (not ST_BRAKE_ENGAGE)
- Modify ST_FAULT_RECOVERY:
  - Change success transition: → ST_FAULT_IDLE (not ST_BRAKE_ENGAGE)
- Add ST_FAULT_IDLE state:
  - Engage brake
  - Disable drive
  - Await fault reset handshake
  - Exit to ST_IDLE on successful handshake

**New Fault Flow**:
```
ST_FAULT (no brake, MC_Halt, output fault code)
  ├─ Handshake → ST_FAULT_RECOVERY → ST_FAULT_IDLE
  └─ Timeout → ST_FAULT_IDLE

ST_FAULT_IDLE (brake engaged, drive off, await handshake)
  └─ Handshake → ST_IDLE
```

---

### Change 3: EOT Fast Jog Torque Limit
**File**: PRG_Main.st

**Problem**: ST_HOME_EOT_FAST uses MC_MoveVelocity with no torque limit; risk of damage if approach overshoots.

**Solution**:
- Replace `fbMoveVelocity` with `fbDirectControl` in velocity mode
- Add torque limit: `Torque := cfgTorqueHomingLimit`

---

### Change 4: FB_PistonExitGuard Simplification (CRITICAL)
**Files**: FB_PistonExitGuard.st, PRG_Main.st

**Problems**:
1. Sign convention WRONG: Comments say "negative torque = pushing toward exit"
   - CORRECT: Pressure pushes toward exit, motor RESISTS with POSITIVE torque
2. Mode-dependent logic is fragile
3. Command correction not wanted

**Solution** - Complete rewrite:
- Remove inputs: CurrentMode, CommandedPosition, CommandedVelocity, CommandedTorque
- Remove outputs: CorrectedPosition, CorrectedVelocity, CorrectedTorque, PositionLimited, VelocityHalted, TorqueAnomalyDetect
- Keep inputs: Enable, ActualPosition, ActualVelocity, ActualTorque, SoftLimitMin, GuardMargin, VelocityThreshold, TorqueThreshold
- Keep outputs: FaultTriggered, GuardActive, InDangerZone, ApproachingLimit

**Simplified Detection Logic**:
```
IF InDangerZone AND
   (ActualTorque > TorqueThreshold) AND      (* Motor resisting with positive torque *)
   (ActualVelocity < -VelocityThreshold) THEN (* Moving toward exit anyway *)
    FaultTriggered := TRUE;  (* Pressure overcoming motor *)
END_IF
```

---

### Change 5: FB_SafetyMonitor Warning Removal
**Files**: FB_SafetyMonitor.st, PRG_Main.st

**Problem**: Warnings not used, adds complexity.

**Solution**:
- Remove outputs: WarnNearLimitMin, WarnNearLimitMax, WarnLowBattery
- Remove input: PositionMargin
- Remove all warning logic (lines 173-175, 208-212)

---

## Files to Modify

| File | Changes |
|------|---------|
| `src/DUT/E_SystemState.st` | Add ST_FAULT_IDLE := 92 |
| `src/GVL/GVL_System.st` | Add posEOTOffset, posActualEOT variables |
| `src/FB/FB_PistonExitGuard.st` | Complete rewrite: fix sign, remove mode logic, detect-only |
| `src/FB/FB_SafetyMonitor.st` | Remove warning outputs/logic, remove PositionMargin |
| `src/PRG/PRG_Main.st` | Fault states, EOT offset, EOT torque limit, FB calls |

---

## Implementation Order

### Phase 1: Safety Critical (First)
1. **FB_PistonExitGuard.st** - Fix sign convention bug (CRITICAL)
2. **E_SystemState.st** - Add ST_FAULT_IDLE
3. **PRG_Main.st** - Fault state architecture (ST_FAULT, ST_FAULT_RECOVERY, add ST_FAULT_IDLE)

### Phase 2: Functional
4. **GVL_System.st** - Add offset variables
5. **PRG_Main.st** - ST_HOME_EOT_SETREF offset calculation (remove fbSetPosition)
6. **PRG_Main.st** - Apply offset to position commands and feedback
7. **PRG_Main.st** - ST_HOME_EOT_FAST torque limit

### Phase 3: Cleanup
8. **FB_SafetyMonitor.st** - Remove warnings
9. **PRG_Main.st** - Update FB calls (SafetyMonitor, PistonExitGuard)

---

## Testing Checklist

### Piston Exit Guard
- [ ] Position = 2mm, Torque = +60%, Velocity = -5mm/s → FaultTriggered = TRUE
- [ ] Position = 2mm, Torque = -60%, Velocity = +5mm/s → FaultTriggered = FALSE
- [ ] Position = -1mm (any torque/velocity) → FaultTriggered = TRUE

### Fault States
- [ ] ST_FAULT: brake NOT engaged (doBrakeDisengage = TRUE)
- [ ] ST_FAULT timeout → ST_FAULT_IDLE (not ST_IDLE)
- [ ] ST_FAULT_IDLE: brake engaged, drive disabled
- [ ] ST_FAULT_IDLE handshake → ST_IDLE

### Dual Coordinate System
- [ ] Mode 110: sysActualPosition = 0 at limit switch
- [ ] Mode 111: posEOTOffset calculated correctly
- [ ] Position commands: master 150mm → hardware = 150mm - offset
- [ ] Position feedback: hardware 150mm → master = 150mm + offset

### EOT Fast Jog
- [ ] Torque limited during ST_HOME_EOT_FAST
- [ ] Smooth transition to ST_HOME_EOT_SLOW
