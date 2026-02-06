(* ============================================================================
   FB_GoHome - Go Home Mode Handler (Mode 101)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Handles Mode 101 (Go Home) which moves the axis to the home position
   (at the home limit switch). This mode checks homing requirements before
   executing and redirects to homing mode if needed.

   Behavior:
   - If AbsHomeRequired is TRUE: Set RedirectToHoming, PRG_Main switches to 110
   - If requirements met: Execute MC_MoveAbsolute to HomePosition
   - After reaching position: Hold there until mode change

   Note: EOTHomeRequired does NOT block Go Home - the axis can still move
   to the home position even if EOT homing hasn't been done. Only
   AbsHomeRequired (encoder validity) blocks the move.

   Integration:
   - Called from ST_GO_HOME state in PRG_Main
   - RedirectToHoming signals PRG_Main to switch to ST_HOME_LIMIT
   - Abort input for motion enable drop handling

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block *)
       Execute             : BOOL             (* Start go home sequence *)
       Axis                : AXIS_REF         (* PLCopen axis reference *)
       AbsHomeRequired     : BOOL             (* Encoder homing required (blocks Go Home) *)
       EOTHomeRequired     : BOOL             (* EOT homing required (does NOT block) *)
       HomePosition        : LREAL            (* Target home position (mm) *)
       HomeVelocity        : LREAL            (* Movement velocity (mm/s) *)
       Abort               : BOOL             (* Abort the move *)
     VAR_OUTPUT:
       Active              : BOOL             (* Sequence is running *)
       Done                : BOOL             (* At home position *)
       AtPosition          : BOOL             (* Currently at home position *)
       HomingRequired      : BOOL             (* Homing must be done first *)
       RedirectToHoming    : BOOL             (* Caller should switch to homing mode *)
       InMotion            : BOOL             (* Currently moving *)
       Error               : BOOL             (* Error occurred *)
       ErrorID             : DWORD            (* Error code if Error=TRUE *)
     VAR:
       fbMoveAbsolute      : MC_MoveAbsolute
       fbHalt              : MC_Halt
       eState              : INT := 0         (* State machine *)
       rTrigExecute        : R_TRIG
       rPosTolerance       : LREAL := 0.1     (* mm *)
     VAR CONSTANT:
       GOH_IDLE            : INT := 0
       GOH_CHECK_REQ       : INT := 1
       GOH_REDIRECT        : INT := 2
       GOH_MOVING          : INT := 3
       GOH_AT_POSITION     : INT := 4
       GOH_ABORTING        : INT := 5
       GOH_ERROR           : INT := 6
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    Active := FALSE;
    Done := FALSE;
    AtPosition := FALSE;
    HomingRequired := FALSE;
    RedirectToHoming := FALSE;
    InMotion := FALSE;
    Error := FALSE;
    ErrorID := 0;
    eState := GOH_IDLE;
    fbMoveAbsolute(Execute := FALSE);
    fbHalt(Execute := FALSE);
    RETURN;
END_IF;

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in any state *)
IF Abort AND (eState <> GOH_IDLE) AND (eState <> GOH_ERROR) AND (eState <> GOH_REDIRECT) THEN
    fbMoveAbsolute(Execute := FALSE);
    eState := GOH_ABORTING;
END_IF;

CASE eState OF

    GOH_IDLE:
        (* Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        AtPosition := FALSE;
        HomingRequired := FALSE;
        RedirectToHoming := FALSE;
        InMotion := FALSE;
        Error := FALSE;
        ErrorID := 0;

        (* Reset function blocks *)
        fbMoveAbsolute(Execute := FALSE);
        fbHalt(Execute := FALSE);

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := GOH_CHECK_REQ;
        END_IF;


    GOH_CHECK_REQ:
        (* Check homing requirements *)

        IF AbsHomeRequired THEN
            (* Encoder homing required - cannot go home without valid position *)
            HomingRequired := TRUE;
            RedirectToHoming := TRUE;
            eState := GOH_REDIRECT;

        ELSE
            (* Requirements met (or only EOT homing needed) - proceed with move *)
            HomingRequired := FALSE;
            RedirectToHoming := FALSE;
            eState := GOH_MOVING;
        END_IF;


    GOH_REDIRECT:
        (* Waiting for PRG_Main to handle redirect *)
        (* Stay in this state - PRG_Main will transition to ST_HOME_LIMIT *)
        (* This FB will be disabled when state changes *)
        Active := TRUE;
        HomingRequired := TRUE;
        RedirectToHoming := TRUE;


    GOH_MOVING:
        (* Execute move to home position *)
        fbMoveAbsolute(
            Axis := Axis,
            Execute := TRUE,
            Position := HomePosition,
            Velocity := HomeVelocity,
            Acceleration := 0.0,    (* Use axis default *)
            Deceleration := 0.0,    (* Use axis default *)
            Jerk := 0.0             (* Use axis default *)
        );

        InMotion := fbMoveAbsolute.Busy;

        IF fbMoveAbsolute.Done THEN
            (* Move complete - reset FB for next use
               NOTE: Setting Execute := FALSE does NOT release position control.
               Per PLCopen standard, axis remains in "Standstill" state with
               active servo position holding. This is standard practice. *)
            fbMoveAbsolute(Execute := FALSE);
            Done := TRUE;
            AtPosition := TRUE;
            InMotion := FALSE;
            eState := GOH_AT_POSITION;

        ELSIF fbMoveAbsolute.Error THEN
            (* Move failed *)
            Error := TRUE;
            ErrorID := fbMoveAbsolute.ErrorID;
            fbMoveAbsolute(Execute := FALSE);
            eState := GOH_ERROR;

        ELSIF fbMoveAbsolute.CommandAborted THEN
            (* Move was aborted externally *)
            fbMoveAbsolute(Execute := FALSE);
            eState := GOH_ABORTING;
        END_IF;


    GOH_AT_POSITION:
        (* At home position - hold here until mode change *)
        Done := TRUE;
        AtPosition := TRUE;
        InMotion := FALSE;
        Active := TRUE;

        (* Check if still at position (could drift in some cases) *)
        (* Note: In practice, the drive holds position in servo mode *)

        (* If Execute goes low, we can reset for next time *)
        IF NOT Execute THEN
            Done := FALSE;
            eState := GOH_IDLE;
        END_IF;


    GOH_ABORTING:
        (* Aborting move - execute halt *)
        fbHalt(
            Axis := Axis,
            Execute := TRUE
        );

        IF fbHalt.Done THEN
            fbHalt(Execute := FALSE);
            Active := FALSE;
            InMotion := FALSE;
            eState := GOH_IDLE;

        ELSIF fbHalt.Error THEN
            (* Halt failed - still try to go to idle *)
            fbHalt(Execute := FALSE);
            eState := GOH_ERROR;
        END_IF;


    GOH_ERROR:
        (* Error state *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        (* Allow reset when Execute goes low *)
        IF NOT Execute THEN
            Error := FALSE;
            ErrorID := 0;
            eState := GOH_IDLE;
        END_IF;

END_CASE;
