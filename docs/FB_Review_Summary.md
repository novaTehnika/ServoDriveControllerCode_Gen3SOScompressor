# Function Block Review Summary
## Date: 2025-12-29

This document summarizes the agent reviews of all function blocks in src/FB/.

---

## FB_AnalogInputFilter.st
**Status**: Minor issues

### Findings:
1. **LOW**: `bInitialized` flag not reset when `Enable` goes FALSE - filter state persists across disable/enable cycles
2. **LOW**: Median index synchronization - `nMedianIndex` wraps correctly but sample overwriting order may cause brief transients

### Recommendations:
- Consider resetting `bInitialized` in the `NOT Enable` branch if fresh start desired
- Current behavior is acceptable for most applications

---

## FB_AnalogProcessor.st
**Status**: Issues found

### Findings:
1. **HIGH**: Stage boundary comparison at line 107 uses `<=` instead of `<`
   - Current: `ELSIF rPosActual <= rStage1Max THEN`
   - Should be: `ELSIF rPosActual < rStage1Max THEN`
   - This causes boundary value to be processed by both stages

2. **HIGH**: No voltage clamping on output
   - `rPositionVoltage` could exceed -10V to +10V range if position exceeds limits
   - Should clamp output to valid DAC range

3. **MEDIUM**: Missing check for `rStage1Max >= rStage2Min` overlap condition

### Recommendations:
- Fix boundary comparison operator
- Add output clamping: `rPositionVoltage := LIMIT(-10.0, rPositionVoltage, 10.0);`

---

## FB_DigitalInputFilter.st
**Status**: Minor issue

### Findings:
1. **MEDIUM**: First scan initialization issue
   - If `RawInput` is already TRUE on first scan, the filter may not immediately recognize this
   - `bLastRaw` initializes to FALSE, so first scan with TRUE input starts debounce timer
   - FilteredOutput stays FALSE until debounce completes (expected behavior)

2. **LOW**: `bPendingState` not explicitly initialized
   - Defaults to FALSE, which is correct for most cases

### Recommendations:
- Add initialization logic if immediate recognition of pre-existing TRUE state is required
- Current behavior is correct for most debounce applications

---

## FB_HandshakeManager.st
**Status**: Critical fix needed

### Findings:
1. **CRITICAL**: Timer not called in HS_WAIT_MOTION_EN state
   - Timer started on transition INTO state but never called while IN state
   - `tTimeout.Q` will never go TRUE because timer isn't being evaluated
   - Location: Lines 162-179

   Fix: Add `tTimeout();` or `tTimeout(IN := TRUE, PT := HandshakeTimeout);` at start of HS_WAIT_MOTION_EN case

2. **LOW**: `eModeAtStart` persists across handshake cycles (cosmetic, no functional impact)

### Recommendations:
- **MUST FIX**: Add timer call in HS_WAIT_MOTION_EN state

---

## FB_ModeDecoder.st
**Status**: Critical fix needed

### Findings:
1. **CRITICAL**: Edge detection logic error at line 69
   - Current: `ModeChanged := NOT ModeStable AND bPrevStable;`
   - This only fires when transitioning FROM stable TO unstable
   - Should detect when mode VALUE changes, not stability state

   Fix: `ModeChanged := (nCurrentValue <> nPreviousValue);`

2. **LOW**: `nPreviousValue` initialized to 0 implicitly (correct, matches MODE_IDLE)

### Recommendations:
- **MUST FIX**: Change ModeChanged detection logic

---

## FB_OutputMux.st
**Status**: Production ready

### Findings:
- No bugs found
- Clean implementation with proper enable handling
- Correct use of centralized FaultToBits conversion
- Output multiplexing logic is correct

### Recommendations:
- None required

---

## FB_PositionMapping.st
**Status**: Minor issue

### Findings:
1. **MEDIUM**: Out-of-range position produces misleading status flags
   - When position is outside both stages, `InStage1` and `InStage2` are both FALSE
   - `MappedVoltage` behavior undefined in this case

2. **LOW**: No explicit clamping of output voltage

### Recommendations:
- Consider adding `OutOfRange` status flag
- Add output voltage clamping for safety

---

## Priority Fixes (Ordered by Severity)

### CRITICAL:
1. **FB_HandshakeManager**: Add timer call in HS_WAIT_MOTION_EN state
2. **FB_ModeDecoder**: Fix ModeChanged edge detection logic

### HIGH:
3. **FB_AnalogProcessor**: Fix stage boundary comparison (`<=` to `<`)
4. **FB_AnalogProcessor**: Add output voltage clamping

### MEDIUM:
5. **FB_DigitalInputFilter**: Consider initialization handling
6. **FB_PositionMapping**: Add out-of-range handling

### LOW:
7. Various initialization and cosmetic issues (acceptable as-is)

---

## FB_PistonExitGuard.st
**Status**: Rewritten (2025-12-30)

### Changes Made:
1. **CRITICAL FIX**: Corrected torque sign convention
   - Previous: Assumed negative torque = pushing toward exit (WRONG)
   - Corrected: Positive torque = motor resisting exit pressure (CORRECT)

2. **Simplified to detect-only**: Removed all command correction logic
   - Removed inputs: CurrentMode, CommandedPosition, CommandedVelocity, CommandedTorque
   - Removed outputs: CorrectedPosition, CorrectedVelocity, CorrectedTorque, PositionLimited, VelocityHalted, TorqueAnomalyDetect
   - Now mode-independent, called once in safety monitoring section

3. **Detection logic**:
   - Triggers fault when: InDangerZone AND (ActualTorque > threshold) AND (ActualVelocity < -threshold)
   - This detects: Motor applying positive torque (resisting) but piston still moving toward exit = pressure overcoming motor

### Recommendations:
- Production ready after rewrite

---

## FB_SafetyMonitor.st
**Status**: Simplified (2025-12-30)

### Changes Made:
1. **Removed warning outputs** (not used, added complexity):
   - Removed: WarnNearLimitMin, WarnNearLimitMax, WarnLowBattery
   - Removed input: PositionMargin

2. **Updated fault state detection**:
   - Added ST_FAULT_IDLE to bInFaultState check
   - Updated fault latch reset to trigger on ST_FAULT_IDLE instead of ST_FAULT_RECOVERY

### Recommendations:
- Production ready after simplification
