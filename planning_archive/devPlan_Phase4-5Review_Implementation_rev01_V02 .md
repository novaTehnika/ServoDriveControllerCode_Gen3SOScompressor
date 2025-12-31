# Plan: Phase 4-5 Rev01 Review Implementation

**Based on**: notes_phase4-5_rev01.md
**Status**: READY FOR APPROVAL

---

## Summary of Changes

| # | Item | Impact | Files |
|---|------|--------|-------|
| 1 | FB_AnalogInputFilter alpha parameterization | Low | FB_AnalogInputFilter.st |
| 2 | Master/Slave terminology (replace User/Hardware) | Medium | GVL_System.st, PRG_Main.st |
| 3 | PositionOutput terminology (replace Feedback) | Medium | GVL_IO.st, FB_PositionMapping.st → FB_PositionOutput.st, PRG_Main.st |
| 4 | ST_HOLD_POSITION ordering verification | Low | Documentation only |
| 5 | ST_BRAKE_HOLD handshake logic | Medium | PRG_Main.st |
| 6 | ST_GO_HOME IF block integration | Low | PRG_Main.st |
| 7 | Limit switch homing over-travel handling | Medium | PRG_Main.st (or FB_HomeLimit) |
| 8 | Encapsulate homing into FBs | High | New FB_HomeLimit.st, FB_HomeEOT.st, PRG_Main.st, E_SystemState.st |
| 9 | MC_Halt/MC_MoveAbsolute Execute:=FALSE verification | Low | Documentation only |

---

## Implementation Phases

### Phase A: Verification & Documentation (Items 4, 9)

**Item 4: ST_HOLD_POSITION ordering** - Current order is CORRECT:
```st
fbHalt(Execute := TRUE);           // FIRST - initiate controlled deceleration
fbDirectControl(Enable := FALSE);  // SECOND - release Y_DirectControl
```
**Rationale:** Y_DirectControl (Yaskawa-specific) releases servo control when disabled. MC_Halt transitions axis to PLCopen "Standstill" state which maintains active position control. Order matters because:
- If Y_DirectControl disabled first → servo loses reference while moving → potential fault
- fbHalt must take over position control BEFORE Y_DirectControl releases it

Add clarifying comment to code explaining this.

**Item 9: MC_Halt/MC_MoveAbsolute Execute:=FALSE behavior**

| FB | When Execute := FALSE | Position Control |
|----|-----------------------|------------------|
| MC_Halt | Axis stays in Standstill | **MAINTAINED** |
| MC_MoveAbsolute | Axis stays in Standstill after Done | **MAINTAINED** |
| Y_DirectControl | Control released | **LOST** |

Per PLCopen standard, axis remains in "Standstill" state when Execute:=FALSE. Servo maintains active position holding. Setting Execute:=FALSE is standard practice to reset the FB for next use.

**Conclusion:** No code changes needed. The pattern `fbMoveAbsolute(Execute := FALSE)` after Done is correct. Lines `fbGoHome(Enable := FALSE)` are appropriate.

---

### Phase B: Simple Refactoring (Items 1, 6)

**Item 1: FB_AnalogInputFilter Alpha Parameterization**
- Remove `CycleTime` input parameter
- Read cycle time from Yaskawa system (reference product manual for SYS_GetTaskTime or equivalent)
- Calculate alpha once during initialization
- Add optional `AlphaOverride` input for testing

**Item 6: ST_GO_HOME IF Block Integration**
```st
// BEFORE: Two separate IF blocks
IF fbGoHome.RedirectToHoming THEN ...
ELSIF fbGoHome.Done THEN ...
ELSIF fbGoHome.Error THEN ...
END_IF
IF fTrigMotionEnable.Q AND NOT fbGoHome.RedirectToHoming THEN ...

// AFTER: Single integrated block
IF fbGoHome.RedirectToHoming THEN ...
ELSIF fbGoHome.Done THEN ...
ELSIF fbGoHome.Error THEN ...
ELSIF fTrigMotionEnable.Q THEN  // Guard not needed - ELSIF ensures RedirectToHoming is FALSE
    fbGoHome(Enable := FALSE);
    sysCurrentState := ST_HOLD_POSITION;
END_IF
```

---

### Phase C: Terminology Updates (Items 2, 3)

**Item 2: Master/Slave Terminology**

| Current | New |
|---------|-----|
| user coordinates | master coordinates |
| hardware coordinates | slave coordinates |
| User → Hardware | Master → Slave |
| Hardware → User | Slave → Master |

Files: GVL_System.st (lines 37-49), PRG_Main.st (multiple locations)

**Item 3: PositionOutput Terminology**

| Current | New |
|---------|-----|
| aoPositionFeedback | aoPositionOutput |
| fbPosMapping | fbPosOutput |
| FB_PositionMapping.st | FB_PositionOutput.st |

Files: GVL_IO.st, FB_PositionMapping.st (rename), PRG_Main.st

---

### Phase D: Functional Enhancements (Items 5, 7)

**Item 5: ST_BRAKE_HOLD Handshake Logic**

Add handshake handling to allow mode transitions:
```st
ST_BRAKE_HOLD:
    IF bEnterState THEN
        doBrakeDisengage := FALSE;
        sysConfirmedMode := MODE_BRAKE_HOLD;
    END_IF

    doInMotion := FALSE;

    (* Add handshake manager call *)
    fbHandshake(...);

    IF fbHandshake.HandshakeComplete THEN
        CASE sysRequestedMode OF
            MODE_IDLE: sysCurrentState := ST_DRIVE_DISABLE;
            MODE_BRAKE_HOLD: ; (* Stay *)
            ELSE: sysCurrentState := ST_BRAKE_RELEASE; (* Release brake for other modes *)
        END_CASE
    END_IF
```

**Item 7: Limit Switch Homing Over-Travel Handling**

If retract limit switch triggers first during approach, reverse direction:
- Add `bApproachFromPositive : BOOL` variable
- If `sysLimitRetractActive` triggers, set flag and approach from positive direction
- This will be incorporated into FB_HomeLimit in Phase E

---

### Phase E: Major Refactoring - Homing FBs (Item 8)

Create two new function blocks following FB_GoHome pattern:

**FB_HomeLimit.st** (Mode 110 - Home to Limit Switch)
```st
FUNCTION_BLOCK FB_HomeLimit
VAR_INPUT
    Enable, Execute     : BOOL;
    Axis                : AXIS_REF;
    ApproachVelocity    : LREAL;
    BackoffVelocity     : LREAL;
    SetPosition         : LREAL;
    Timeout             : TIME;
    LimitHomeActive     : BOOL;
    LimitRetractActive  : BOOL;  (* For over-travel detection *)
    Abort               : BOOL;
END_VAR
VAR_OUTPUT
    Active, Done, InMotion, Error : BOOL;
    ErrorID             : DWORD;
END_VAR
```
Internal states: HL_IDLE → HL_APPROACH → HL_REVERSE (if needed) → HL_DETECT → HL_BACKOFF → HL_SETREF → HL_DONE

**FB_HomeEOT.st** (Mode 111 - Home to End-of-Travel)
```st
FUNCTION_BLOCK FB_HomeEOT
VAR_INPUT
    Enable, Execute     : BOOL;
    Axis                : AXIS_REF;
    AbsHomeRequired     : BOOL;  (* Error if TRUE *)
    FastVelocity, SlowVelocity : LREAL;
    TorqueLimit, TorqueThreshold : LREAL;
    ApproachDistance    : LREAL;
    ExpectedEOTPosition : LREAL;
    StallDetectTime, Timeout : TIME;
    ActualPosition, ActualVelocity, ActualTorque : LREAL;
    Abort               : BOOL;
END_VAR
VAR_OUTPUT
    Active, Done, InMotion, Error : BOOL;
    EOTPosition         : LREAL;  (* Recorded stall position *)
    EOTOffset           : LREAL;  (* Calculated offset *)
    RequiresLimitHome   : BOOL;
END_VAR
```
Internal states: HE_IDLE → HE_CHECK_REQ → HE_FAST_APPROACH → HE_SLOW_APPROACH → HE_STALL_DETECT → HE_SETREF → HE_DONE

**PRG_Main Simplification** - Replace ~150 lines of homing sub-states with:
```st
ST_HOME_LIMIT:
    fbHomeLimit(Enable := TRUE, Execute := TRUE, ...);
    IF fbHomeLimit.Done THEN
        flagAbsHomeRequired := FALSE;
        sysCurrentState := ST_HOME_COMPLETE;
    ELSIF fbHomeLimit.Error THEN ...
    ELSIF fTrigMotionEnable.Q THEN ...
    END_IF

ST_HOME_EOT:
    fbHomeEOT(Enable := TRUE, Execute := TRUE, ...);
    IF fbHomeEOT.Done THEN
        posActualEOT := fbHomeEOT.EOTPosition;
        posEOTOffset := fbHomeEOT.EOTOffset;
        flagEOTHomeRequired := FALSE;
        sysCurrentState := ST_HOME_COMPLETE;
    ELSIF ...
    END_IF
```

**E_SystemState.st** - Mark sub-states as deprecated (keep values for compatibility):
- ST_HOME_LIM_APPROACH (43) - deprecated
- ST_HOME_LIM_DETECT (44) - deprecated
- ST_HOME_LIM_BACKOFF (45) - deprecated
- ST_HOME_LIM_SETREF (46) - deprecated
- ST_HOME_EOT_FAST (50) - deprecated
- ST_HOME_EOT_SLOW (51) - deprecated
- ST_HOME_EOT_DETECT (52) - deprecated
- ST_HOME_EOT_SETREF (53) - deprecated

---

## Files to Modify/Create

| File | Action | Phase |
|------|--------|-------|
| src/FB/FB_AnalogInputFilter.st | Modify | B |
| src/PRG/PRG_Main.st | Modify | B, C, D, E |
| src/GVL/GVL_System.st | Modify | C |
| src/GVL/GVL_IO.st | Modify | C |
| src/FB/FB_PositionMapping.st | Rename to FB_PositionOutput.st | C |
| src/FB/FB_HomeLimit.st | **Create** | E |
| src/FB/FB_HomeEOT.st | **Create** | E |
| src/DUT/E_SystemState.st | Modify | E |
| docs/diagrams/*.puml | Update | E |

---

## Implementation Order

1. **Phase A** (Items 4, 9) - Verification, add comments
2. **Phase B** (Items 1, 6) - Simple refactoring
3. **Phase C** (Items 2, 3) - Terminology updates
4. **Phase D** (Items 5, 7) - Functional enhancements
5. **Phase E** (Item 8) - Major homing refactoring

---

## Testing Checklist

- [ ] FB_AnalogInputFilter: Alpha calculation correct for various cycle times
- [ ] ST_BRAKE_HOLD: Can transition to other modes via handshake
- [ ] ST_GO_HOME: Motion enable drop handled correctly
- [ ] Limit homing: Over-travel reversal works
- [ ] FB_HomeLimit: All sub-states work, timeout/abort handling
- [ ] FB_HomeEOT: Stall detection, offset calculation correct
- [ ] Terminology: Build succeeds, no functional changes
- [ ] Regression: All existing modes still function
