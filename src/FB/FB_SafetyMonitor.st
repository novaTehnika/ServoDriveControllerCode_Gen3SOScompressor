(* ============================================================================
   FB_SafetyMonitor - Centralized Safety Monitoring

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Monitors all safety-related conditions and generates appropriate fault codes.
   This function block provides a single point for all safety monitoring,
   making the system easier to audit and maintain.

   Monitored Conditions:
   - Drive faults (from servo amplifier; encoder alarms surface here via the drive)
   - Limit switch faults (unexpected triggers during motion)
   - Position limit violations (beyond soft limits)
   - Piston exit guard faults (from FB_PistonExitGuard)

   Fault Priority (highest to lowest):
   1. Drive fault (FAULT_DRIVE)
   2. Limit switch fault (FAULT_LIMIT_SWITCH)
   3. Position limit fault (FAULT_POSITION)
   4. Piston exit fault (FAULT_PISTON_EXIT)

   Integration:
   - Called every scan after state machine, before output update
   - FaultDetected triggers transition to E_SystemState#ST_FAULT in PRG_Main

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable safety monitoring
       LimitRetractActive  : BOOL             (* TRUE = retract limit triggered
       LimitHomeActive     : BOOL             (* TRUE = home limit triggered
       ActualPosition      : LREAL            (* Current axis position (mm)
       SoftLimitMin        : LREAL            (* Minimum allowed position (mm)
       SoftLimitMax        : LREAL            (* Maximum allowed position (mm)
       DriveReady          : BOOL             (* Drive is ready for motion
       DriveFault          : BOOL             (* Drive has reported a fault
       CurrentState        : E_SystemState    (* Current state machine state
       CurrentMode         : E_OperatingMode  (* Current operating mode
       ExitGuardTriggered  : BOOL             (* Piston exit guard triggered fault
     VAR_OUTPUT:
       FaultDetected       : BOOL             (* A fault condition exists
       FaultCode           : E_FaultCode      (* Fault code to report
       FaultLimitSwitch    : BOOL             (* Limit switch fault detected
       FaultPosition       : BOOL             (* Position limit exceeded
       FaultDrive          : BOOL             (* Drive fault detected
       FaultPistonExit     : BOOL             (* Piston exit guard fault
     VAR:
       bLatchedFault       : BOOL
       eLatchedFaultCode   : E_FaultCode
       bInMotionState      : BOOL
       bInHomingState      : BOOL
       bInFaultState       : BOOL
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    FaultDetected := FALSE;
    FaultCode := E_FaultCode#FAULT_NONE;
    FaultLimitSwitch := FALSE;
    FaultPosition := FALSE;
    FaultDrive := FALSE;
    FaultPistonExit := FALSE;
    bLatchedFault := FALSE;
    eLatchedFaultCode := E_FaultCode#FAULT_NONE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Determine operating context
   -------------------------------------------------------------------------- *)

(* Check if in a motion state *)
bInMotionState := (CurrentState = E_SystemState#ST_POSITION_CTRL) OR
                  (CurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
                  (CurrentState = E_SystemState#ST_TORQUE_CTRL) OR
                  (CurrentState = E_SystemState#ST_GO_HOME) OR
                  (CurrentState = E_SystemState#ST_HOLD_POSITION);

(* Check if in a homing state *)
bInHomingState := (CurrentState = E_SystemState#ST_HOME_LIMIT) OR
                  (CurrentState = E_SystemState#ST_HOME_EOT) OR
                  (CurrentState = E_SystemState#ST_HOME_COMPLETE);

(* Check if already in fault state *)
bInFaultState := (CurrentState = E_SystemState#ST_FAULT) OR
                 (CurrentState = E_SystemState#ST_FAULT_IDLE);

(* --------------------------------------------------------------------------
   Reset fault latch when appropriate
   -------------------------------------------------------------------------- *)

(* Clear fault latch when entering fault idle or returning to idle *)
IF (CurrentState = E_SystemState#ST_FAULT_IDLE) OR (CurrentState = E_SystemState#ST_IDLE) THEN
    IF NOT FaultDetected THEN
        bLatchedFault := FALSE;
        eLatchedFaultCode := E_FaultCode#FAULT_NONE;
    END_IF;
END_IF;

(* --------------------------------------------------------------------------
   DRIVE FAULT MONITORING
   Most critical - servo amplifier has detected an issue
   -------------------------------------------------------------------------- *)

FaultDrive := DriveFault;

(* --------------------------------------------------------------------------
   LIMIT SWITCH FAULT MONITORING
   Unexpected limit switch activation during motion
   -------------------------------------------------------------------------- *)

(* Limit switches are expected during homing - only fault if unexpected *)
IF bInMotionState AND NOT bInHomingState THEN
    (* During normal motion, hitting a limit switch is unexpected *)
    (* This could indicate mechanical issue, runaway, or position error *)
    IF LimitRetractActive OR LimitHomeActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSE
    (* Not in motion or in homing - limits are OK or expected *)
    FaultLimitSwitch := FALSE;
END_IF;

(* --------------------------------------------------------------------------
   POSITION LIMIT MONITORING
   Position beyond software limits
   -------------------------------------------------------------------------- *)

(* Check if position is outside software limits *)
IF (ActualPosition < SoftLimitMin) OR (ActualPosition > SoftLimitMax) THEN
    FaultPosition := TRUE;
ELSE
    FaultPosition := FALSE;
END_IF;

(* --------------------------------------------------------------------------
   PISTON EXIT GUARD MONITORING
   Triggered by FB_PistonExitGuard
   -------------------------------------------------------------------------- *)

FaultPistonExit := ExitGuardTriggered;

(* --------------------------------------------------------------------------
   FAULT PRIORITY AND OUTPUT
   Determine highest priority fault and set outputs
   -------------------------------------------------------------------------- *)

(* Priority order: Drive > LimitSwitch > Position > PistonExit *)
IF FaultDrive THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_DRIVE;

ELSIF FaultLimitSwitch THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_LIMIT_SWITCH;

ELSIF FaultPosition THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_POSITION;

ELSIF FaultPistonExit THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_PISTON_EXIT;

ELSE
    FaultDetected := FALSE;
    FaultCode := E_FaultCode#FAULT_NONE;
END_IF;

(* --------------------------------------------------------------------------
   FAULT LATCHING
   Latch fault for persistence across scans until explicitly cleared
   -------------------------------------------------------------------------- *)

IF FaultDetected AND NOT bLatchedFault AND NOT bInFaultState THEN
    bLatchedFault := TRUE;
    eLatchedFaultCode := FaultCode;
END_IF;

(* Use latched fault if current scan doesn't detect new fault *)
IF bLatchedFault AND NOT FaultDetected THEN
    FaultDetected := TRUE;
    FaultCode := eLatchedFaultCode;
END_IF;
