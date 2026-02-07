(* ============================================================================
   FB_HomeLimit - Limit Switch Homing Sequence Handler (Mode 110)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Encapsulates the complete limit switch homing sequence for Mode 110.
   This FB handles the entire sequence from approach to setting the position
   reference, including over-travel detection and direction reversal.

   Sequence:
   1. APPROACH: Move toward home limit switch (negative direction first)
      - If retract limit triggers instead, reverse to positive direction
   2. DETECT: Stop motion when home limit switch detected
   3. BACKOFF: Move away from switch at slow speed until switch clears
   4. SETREF: Set position reference at switch edge location

   Over-Travel Handling:
   - If retract limit triggers during approach from negative direction,
     the FB automatically reverses to approach from positive direction
   - If retract limit triggers during positive approach, this is a fault

   Integration:
   - Called from ST_HOME_LIMIT state in PRG_Main
   - Abort input for motion enable drop handling

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       Execute             : BOOL             (* Start homing sequence
       Axis                : AXIS_REF         (* PLCopen axis reference
       ApproachVelocity    : LREAL            (* Approach velocity (mm/s)
       BackoffVelocity     : LREAL            (* Backoff velocity (mm/s)
       SetPosition         : LREAL            (* Position to set at home switch (mm)
       Timeout             : TIME             (* Overall sequence timeout
       LimitHomeActive     : BOOL             (* Home limit switch is triggered
       LimitRetractActive  : BOOL             (* Retract limit switch is triggered
       Abort               : BOOL             (* Abort the sequence
     VAR_OUTPUT:
       Active              : BOOL             (* Sequence is running
       Done                : BOOL             (* Homing complete
       InMotion            : BOOL             (* Currently moving
       Error               : BOOL             (* Error occurred
       ErrorID             : DWORD            (* Error code if Error=TRUE
       ApproachingPositive : BOOL             (* Currently approaching from positive direction
     VAR:
       fbMoveVelocity      : MC_MoveVelocity
       fbStop              : MC_Stop
       fbSetPosition       : MC_SetPosition
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       tTimeout            : TON
       bApproachFromPositive : BOOL := FALSE
     VAR CONSTANT:
       HL_IDLE             : INT := 0
       HL_APPROACH         : INT := 1
       HL_DETECT           : INT := 2
       HL_BACKOFF          : INT := 3
       HL_SETREF           : INT := 4
       HL_DONE             : INT := 5
       HL_ABORTING         : INT := 6
       HL_ERROR            : INT := 7
       ERR_TIMEOUT         : DWORD := 16#8001
       ERR_DRIVE           : DWORD := 16#8002
       ERR_LIMIT_INVALID   : DWORD := 16#8003
       ERR_SETPOS          : DWORD := 16#8004
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
    ErrorID := 0;
    ApproachingPositive := FALSE;
    eState := HL_IDLE;
    bApproachFromPositive := FALSE;
    fbMoveVelocity(Execute := FALSE);
    fbStop(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    tTimeout(IN := FALSE);
    RETURN;
END_IF;

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in most states *)
IF Abort AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND (eState <> HL_DONE) THEN
    fbMoveVelocity(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    tTimeout(IN := FALSE);
    eState := HL_ABORTING;
END_IF;

(* Overall timeout check *)
tTimeout(IN := (eState <> HL_IDLE) AND (eState <> HL_DONE) AND (eState <> HL_ERROR) AND (eState <> HL_ABORTING),
         PT := Timeout);

IF tTimeout.Q THEN
    fbMoveVelocity(Execute := FALSE);
    fbStop(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    Error := TRUE;
    ErrorID := ERR_TIMEOUT;
    eState := HL_ERROR;
END_IF;

(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

IF eState = HL_IDLE THEN

    (* HL_IDLE - Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        InMotion := FALSE;
        Error := FALSE;
        ErrorID := 0;
        ApproachingPositive := FALSE;
        bApproachFromPositive := FALSE;

        (* Reset function blocks *)
        fbMoveVelocity(Execute := FALSE);
        fbStop(Execute := FALSE);
        fbSetPosition(Execute := FALSE);
        tTimeout(IN := FALSE);

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            bApproachFromPositive := FALSE;  (* Start approaching from negative *)
            eState := HL_APPROACH;
        END_IF;


ELSIF eState = HL_APPROACH THEN

    (* HL_APPROACH - Move toward home limit switch
       Direction depends on bApproachFromPositive:
       - FALSE: Approach from negative direction (default)
       - TRUE: Approach from positive direction (after over-travel) *)
        ApproachingPositive := bApproachFromPositive;

        IF bApproachFromPositive THEN
            fbMoveVelocity(
                Axis := Axis,
                Execute := TRUE,
                Velocity := ApproachVelocity,
                Direction := MC_Positive_Direction
            );
        ELSE
            fbMoveVelocity(
                Axis := Axis,
                Execute := TRUE,
                Velocity := ApproachVelocity,
                Direction := MC_Negative_Direction
            );
        END_IF;

        InMotion := fbMoveVelocity.Busy;

        IF LimitHomeActive THEN
            (* Home limit switch triggered - stop and detect *)
            fbMoveVelocity(Execute := FALSE);
            eState := HL_DETECT;

        ELSIF LimitRetractActive THEN
            (* Over-travel: Hit retract limit instead of home limit *)
            IF NOT bApproachFromPositive THEN
                (* Was approaching from negative - reverse to positive direction *)
                fbMoveVelocity(Execute := FALSE);
                bApproachFromPositive := TRUE;
                (* Stay in HL_APPROACH, will re-enter with new direction next scan *)

            ELSE
                (* Already approaching from positive and hit retract - fault *)
                fbMoveVelocity(Execute := FALSE);
                Error := TRUE;
                ErrorID := ERR_LIMIT_INVALID;
                eState := HL_ERROR;
            END_IF;

        ELSIF fbMoveVelocity.Error THEN
            fbMoveVelocity(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_DETECT THEN

    (* HL_DETECT - Stop motion after limit switch detected *)
        fbStop(
            Axis := Axis,
            Execute := TRUE
        );

        InMotion := NOT fbStop.Done;

        IF fbStop.Done THEN
            fbStop(Execute := FALSE);
            eState := HL_BACKOFF;

        ELSIF fbStop.Error THEN
            fbStop(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_BACKOFF THEN

    (* HL_BACKOFF - Move away from limit switch at slow speed until switch clears *)
        fbMoveVelocity(
            Axis := Axis,
            Execute := TRUE,
            Velocity := BackoffVelocity,
            Direction := MC_Positive_Direction
        );

        InMotion := fbMoveVelocity.Busy;

        IF NOT LimitHomeActive THEN
            (* Switch cleared - stop and set position *)
            fbMoveVelocity(Execute := FALSE);
            eState := HL_SETREF;

        ELSIF fbMoveVelocity.Error THEN
            fbMoveVelocity(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_SETREF THEN

    (* HL_SETREF - Set position reference at home switch edge location *)
        (* First stop to ensure fully stopped *)
        fbStop(
            Axis := Axis,
            Execute := NOT fbStop.Done
        );

        IF fbStop.Done THEN
            fbStop(Execute := FALSE);

            (* Set this position as the home reference *)
            fbSetPosition(
                Axis := Axis,
                Execute := TRUE,
                Position := SetPosition
            );

            IF fbSetPosition.Done THEN
                fbSetPosition(Execute := FALSE);
                InMotion := FALSE;
                Done := TRUE;
                eState := HL_DONE;

            ELSIF fbSetPosition.Error THEN
                fbSetPosition(Execute := FALSE);
                Error := TRUE;
                ErrorID := ERR_SETPOS;
                eState := HL_ERROR;
            END_IF;
        END_IF;


ELSIF eState = HL_DONE THEN

    (* HL_DONE - Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;
        ApproachingPositive := FALSE;

        (* Reset when Execute goes low *)
        IF NOT Execute THEN
            Done := FALSE;
            eState := HL_IDLE;
        END_IF;


ELSIF eState = HL_ABORTING THEN

    (* HL_ABORTING - Stop motion *)
        fbStop(
            Axis := Axis,
            Execute := TRUE
        );

        IF fbStop.Done THEN
            fbStop(Execute := FALSE);
            Active := FALSE;
            InMotion := FALSE;
            eState := HL_IDLE;

        ELSIF fbStop.Error THEN
            (* Stop failed - still try to go to idle *)
            fbStop(Execute := FALSE);
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_ERROR THEN

    (* HL_ERROR *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        (* Allow reset when Execute goes low *)
        IF NOT Execute THEN
            Error := FALSE;
            ErrorID := 0;
            eState := HL_IDLE;
        END_IF;

END_IF;
