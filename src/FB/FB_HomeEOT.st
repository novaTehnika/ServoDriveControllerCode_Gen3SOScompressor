(* ============================================================================
   FB_HomeEOT - End-of-Travel Homing Sequence Handler (Mode 111)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Encapsulates the complete End-of-Travel (EOT) homing sequence for Mode 111.
   Uses torque-limited velocity control to find the mechanical end stop, then
   calculates a coordinate offset for master/slave translation.

   Sequence:
   1. CHECK_REQ: Verify AbsHomeRequired is FALSE (limit switch homing must be done first)
   2. FAST_APPROACH: Fast jog toward EOT with torque limiting for safety
   3. SLOW_APPROACH: Slow torque-limited approach when near expected position
   4. STALL_DETECT: Confirm stall condition (velocity near zero, torque at limit)
   5. SETREF: Calculate coordinate offset (preserves encoder zero from Mode 110)

   Coordinate System:
   This FB calculates EOTOffset for master/slave coordinate transformation:
   - Master position = Slave position + EOTOffset
   - EOTOffset = ExpectedEOTPosition - ActualEOTPosition (encoder value at stall)

   This preserves the limit switch zero reference from Mode 110 while allowing
   the master to see the expected position value at EOT.

   Uses Y_DirectControl:
   - Mode 2 (velocity mode) with torque limiting for safe mechanical contact

   Integration:
   - Called from ST_HOME_EOT state in PRG_Main
   - RequiresLimitHome output signals need for Mode 110 first
   - Abort input for motion enable drop handling

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       Execute             : BOOL             (* Start homing sequence
       Axis                : AXIS_REF         (* PLCopen axis reference
       AbsHomeRequired     : BOOL             (* Limit switch homing required
       FastVelocity        : LREAL            (* Fast approach velocity (mm/s)
       SlowVelocity        : LREAL            (* Slow approach velocity (mm/s)
       TorqueLimit         : LREAL            (* Torque limit during approach (%)
       TorqueThreshold     : LREAL            (* Torque threshold for stall detection (%)
       ApproachDistance    : LREAL            (* Distance before EOT to switch to slow (mm)
       ExpectedEOTPosition : LREAL            (* Expected position at EOT (mm)
       StallDetectTime     : TIME             (* Time to confirm stall condition
       Timeout             : TIME             (* Overall sequence timeout
       ActualPosition      : LREAL            (* Current position (mm)
       ActualVelocity      : LREAL            (* Current velocity (mm/s)
       ActualTorque        : LREAL            (* Current torque (% of rated)
       Abort               : BOOL             (* Abort the sequence
     VAR_OUTPUT:
       Active              : BOOL             (* Sequence is running
       Done                : BOOL             (* Homing complete
       InMotion            : BOOL             (* Currently moving
       Error               : BOOL             (* Error occurred
       RequiresLimitHome   : BOOL             (* Must do limit switch homing first
       EOTPosition         : LREAL            (* Recorded encoder position at EOT stall
       EOTOffset           : LREAL            (* Calculated coordinate offset
     VAR:
       fbDirectControl     : Y_DirectControl
       fbStop              : MC_Stop
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       tTimeout            : TON
       tStallConfirm       : TON
       rVelocityThreshold  : LREAL := 0.5
       rTorqueMultiplier   : LREAL := 0.9
     VAR CONSTANT:
       HE_IDLE             : INT := 0
       HE_CHECK_REQ        : INT := 1
       HE_REQUIRES_HOME    : INT := 2
       HE_FAST_APPROACH    : INT := 3
       HE_SLOW_APPROACH    : INT := 4
       HE_STALL_DETECT     : INT := 5
       HE_SETREF           : INT := 6
       HE_DONE             : INT := 7
       HE_ABORTING         : INT := 8
       HE_ERROR            : INT := 9
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    Active := FALSE;
    Done := FALSE;
    InMotion := FALSE;
    Error := FALSE;
    RequiresLimitHome := FALSE;
    EOTPosition := 0.0;
    EOTOffset := 0.0;
    eState := HE_IDLE;
    fbDirectControl(Enable := FALSE);
    fbStop(Execute := FALSE);
    tTimeout(IN := FALSE);
    tStallConfirm(IN := FALSE);
    RETURN;
END_IF;

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in most states *)
IF Abort AND (eState <> HE_IDLE) AND (eState <> HE_ERROR) AND
   (eState <> HE_DONE) AND (eState <> HE_REQUIRES_HOME) THEN
    fbDirectControl(Enable := FALSE);
    tTimeout(IN := FALSE);
    tStallConfirm(IN := FALSE);
    eState := HE_ABORTING;
END_IF;

(* Overall timeout check *)
tTimeout(IN := (eState >= HE_FAST_APPROACH) AND (eState <= HE_SETREF),
         PT := Timeout);

IF tTimeout.Q THEN
    fbDirectControl(Enable := FALSE);
    fbStop(Execute := FALSE);
    Error := TRUE;
    eState := HE_ERROR;
END_IF;

(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

IF eState = HE_IDLE THEN

    (* HE_IDLE - Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        InMotion := FALSE;
        Error := FALSE;
        RequiresLimitHome := FALSE;
        EOTPosition := 0.0;
        EOTOffset := 0.0;

        (* Reset function blocks *)
        fbDirectControl(Enable := FALSE);
        fbStop(Execute := FALSE);
        tTimeout(IN := FALSE);
        tStallConfirm(IN := FALSE);

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := HE_CHECK_REQ;
        END_IF;


ELSIF eState = HE_CHECK_REQ THEN

    (* HE_CHECK_REQ - Check if limit switch homing is required first *)
        IF AbsHomeRequired THEN
            (* Must do limit switch homing before EOT homing *)
            RequiresLimitHome := TRUE;
            eState := HE_REQUIRES_HOME;
        ELSE
            (* Requirements met - proceed to fast approach *)
            RequiresLimitHome := FALSE;
            eState := HE_FAST_APPROACH;
        END_IF;


ELSIF eState = HE_REQUIRES_HOME THEN

    (* HE_REQUIRES_HOME - Waiting for caller to handle the requirement *)
        (* PRG_Main should transition to fault or Mode 110 *)
        Active := TRUE;
        RequiresLimitHome := TRUE;
        (* Note: Caller should disable this FB and handle the error *)


ELSIF eState = HE_FAST_APPROACH THEN

    (* HE_FAST_APPROACH - Fast jog toward EOT (positive/extend direction) with torque limit *)
        fbDirectControl(
            Axis := Axis,
            Enable := TRUE,
            ControlMode := 2,  (* Velocity mode *)
            Velocity := FastVelocity,
            Torque := TorqueLimit
        );

        InMotion := fbDirectControl.Active;

        (* Switch to slow approach when near expected EOT *)
        IF ActualPosition >= (ExpectedEOTPosition - ApproachDistance) THEN
            fbDirectControl(Enable := FALSE);
            eState := HE_SLOW_APPROACH;

        ELSIF fbDirectControl.Error THEN
            fbDirectControl(Enable := FALSE);
            Error := TRUE;
            eState := HE_ERROR;
        END_IF;


ELSIF eState = HE_SLOW_APPROACH THEN

    (* HE_SLOW_APPROACH - Slow torque-limited approach to EOT *)
        fbDirectControl(
            Axis := Axis,
            Enable := TRUE,
            ControlMode := 2,  (* Velocity mode *)
            Velocity := SlowVelocity,
            Torque := TorqueLimit
        );

        InMotion := TRUE;
        tStallConfirm(IN := FALSE);  (* Reset stall timer *)

        (* Detect stall: velocity near zero while torque at limit *)
        IF (ABS(ActualVelocity) < rVelocityThreshold) AND
           (ABS(ActualTorque) >= TorqueThreshold * rTorqueMultiplier) THEN
            (* Stall condition detected - move to confirm *)
            eState := HE_STALL_DETECT;

        ELSIF fbDirectControl.Error THEN
            fbDirectControl(Enable := FALSE);
            Error := TRUE;
            eState := HE_ERROR;
        END_IF;


ELSIF eState = HE_STALL_DETECT THEN

    (* HE_STALL_DETECT - Confirm stall condition for StallDetectTime *)
        (* Continue pushing against EOT to confirm *)
        fbDirectControl(
            Axis := Axis,
            Enable := TRUE,
            ControlMode := 2,
            Velocity := SlowVelocity,
            Torque := TorqueLimit
        );

        InMotion := TRUE;

        (* Timer for stall confirmation *)
        tStallConfirm(IN := TRUE, PT := StallDetectTime);

        (* Check if still stalled *)
        IF (ABS(ActualVelocity) < rVelocityThreshold) AND
           (ABS(ActualTorque) >= TorqueThreshold * rTorqueMultiplier) THEN
            IF tStallConfirm.Q THEN
                (* Stall confirmed - proceed to set reference *)
                fbDirectControl(Enable := FALSE);
                eState := HE_SETREF;
            END_IF;
        ELSE
            (* Not stalled anymore - go back to slow approach *)
            tStallConfirm(IN := FALSE);
            eState := HE_SLOW_APPROACH;
        END_IF;


ELSIF eState = HE_SETREF THEN

    (* HE_SETREF - Calculate EOT offset - DO NOT call MC_SetPosition
       This preserves the limit switch zero reference from Mode 110 *)

        (* First stop to ensure fully stopped *)
        fbStop(
            Axis := Axis,
            Execute := NOT fbStop.Done
        );

        IF fbStop.Done THEN
            fbStop(Execute := FALSE);

            (* Record actual encoder position at EOT stall *)
            EOTPosition := ActualPosition;

            (* Calculate offset for coordinate transformation *)
            (* Master position = Slave position + offset *)
            (* Example: If actual EOT is at encoder 295mm and expected is 300mm, *)
            (*          offset = 300 - 295 = 5mm *)
            (*          Master sees 300mm, slave is at 295mm *)
            EOTOffset := ExpectedEOTPosition - EOTPosition;

            InMotion := FALSE;
            Done := TRUE;
            eState := HE_DONE;
        END_IF;


ELSIF eState = HE_DONE THEN

    (* HE_DONE - Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;

        (* Reset when Execute goes low *)
        IF NOT Execute THEN
            Done := FALSE;
            eState := HE_IDLE;
        END_IF;


ELSIF eState = HE_ABORTING THEN

    (* HE_ABORTING - Stop motion *)
        fbDirectControl(Enable := FALSE);
        fbStop(
            Axis := Axis,
            Execute := TRUE
        );

        IF fbStop.Done THEN
            fbStop(Execute := FALSE);
            Active := FALSE;
            InMotion := FALSE;
            eState := HE_IDLE;

        ELSIF fbStop.Error THEN
            (* Stop failed - still try to go to idle *)
            fbStop(Execute := FALSE);
            eState := HE_ERROR;
        END_IF;


ELSIF eState = HE_ERROR THEN

    (* HE_ERROR *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;
        fbDirectControl(Enable := FALSE);

        (* Allow reset when Execute goes low *)
        IF NOT Execute THEN
            Error := FALSE;
            eState := HE_IDLE;
        END_IF;

END_IF;
