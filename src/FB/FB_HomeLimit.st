(* ============================================================================
   FB_HomeLimit - Home-to-Negative-Overtravel Sequence Handler (Mode 110)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Drives the ram in the negative direction until the negative overtravel
   switch is tripped, backs off until the switch clears, then sets the
   position reference. This establishes the single coordinate frame used
   by the rest of the system.

   Sequence:
   1. APPROACH: MC_MoveVelocity at ApproachVelocity in negative direction.
   2. DETECT:   MC_Stop triggered by OvertravelNegActive.
   3. BACKOFF:  MC_MoveVelocity at BackoffVelocity in positive direction
                until OvertravelNegActive clears.
   4. SETREF:   MC_Stop, then MC_SetPosition(SetPosition).

   Error conditions:
   - OvertravelPosActive at any point during the sequence (homing never
     approaches the positive switch, so this is always a fault).
   - Any built-in motion FB reporting Error.
   - Overall Timeout expiring.

   Integration:
   - Called once per scan at end of PRG_Main.
   - CmdMoveVelocity/CmdStop/CmdSetPosition routed to G_cmd* by PRG_Main.
   - PRG_Main routes G_staMoveVelocity/G_staStop/G_staSetPosition into
     the StaMoveVelocity/StaStop/StaSetPosition inputs.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL                   (* Enable the function block
       Execute             : BOOL                   (* Start homing sequence
       ApproachVelocity    : LREAL                  (* Approach velocity magnitude (mm/s)
       BackoffVelocity     : LREAL                  (* Backoff velocity magnitude (mm/s)
       Acceleration        : LREAL                  (* Move acceleration (mm/s²)
       Deceleration        : LREAL                  (* Move/stop deceleration (mm/s²)
       SetPosition         : LREAL                  (* Position to set at home (mm)
       Timeout             : TIME                   (* Overall sequence timeout
       OvertravelNegActive : BOOL                   (* Negative overtravel / home switch triggered
       OvertravelPosActive : BOOL                   (* Positive overtravel switch triggered (fault)
       Abort               : BOOL                   (* Abort the sequence
       StaMoveVelocity     : ST_StaMoveVelocity     (* Status from MC_MoveVelocity
       StaStop             : ST_StaStop             (* Status from MC_Stop
       StaSetPosition      : ST_StaSetPosition      (* Status from MC_SetPosition
     VAR_OUTPUT:
       Active              : BOOL                   (* Sequence is running
       Done                : BOOL                   (* Homing complete
       InMotion            : BOOL                   (* Currently moving
       Error               : BOOL                   (* Error occurred
       CmdMoveVelocity     : ST_CmdMoveVelocity     (* Command output for MC_MoveVelocity
       CmdStop             : ST_CmdStop             (* Command output for MC_Stop
       CmdSetPosition      : ST_CmdSetPosition     (* Command output for MC_SetPosition
     VAR:
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       tTimeout            : TON              (* Single call site - see end of body
       bTimeoutIN          : BOOL             (* Latched IN for tTimeout
       bStopDoneLatched    : BOOL := FALSE
     VAR CONSTANT:
       HL_IDLE             : INT := 0
       HL_APPROACH         : INT := 1
       HL_DETECT           : INT := 2
       HL_BACKOFF          : INT := 3
       HL_SETREF           : INT := 4
       HL_DONE             : INT := 5
       HL_ABORTING         : INT := 6
       HL_ERROR            : INT := 7
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    Active := FALSE;
    Done := FALSE;
    InMotion := FALSE;
    Error := FALSE;
    eState := HL_IDLE;
    bStopDoneLatched := FALSE;
    CmdMoveVelocity.Execute := FALSE;
    CmdStop.Execute := FALSE;
    CmdSetPosition.Execute := FALSE;
    bTimeoutIN := FALSE;

ELSE
    (* Detect rising edge on Execute *)
    rTrigExecute(CLK := Execute);

    (* Handle abort in most states *)
    IF Abort AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND (eState <> HL_DONE) THEN
        CmdMoveVelocity.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        eState := HL_ABORTING;
    END_IF;

    (* Positive overtravel switch must never trigger during homing. *)
    IF OvertravelPosActive AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND
       (eState <> HL_DONE) AND (eState <> HL_ABORTING) THEN
        CmdMoveVelocity.Execute := FALSE;
        CmdStop.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        Error := TRUE;
        eState := HL_ERROR;
    END_IF;

    (* Default timer IN based on state; overall timeout check uses last-scan Q *)
    bTimeoutIN := (eState <> HL_IDLE) AND (eState <> HL_DONE) AND (eState <> HL_ERROR) AND (eState <> HL_ABORTING);

    IF tTimeout.Q THEN
        CmdMoveVelocity.Execute := FALSE;
        CmdStop.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        Error := TRUE;
        bTimeoutIN := FALSE;
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
        bStopDoneLatched := FALSE;

        CmdMoveVelocity.Execute := FALSE;
        CmdStop.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := HL_APPROACH;
        END_IF;


ELSIF eState = HL_APPROACH THEN

    (* HL_APPROACH - Move negative toward overtravel / home switch *)
        CmdMoveVelocity.Execute := TRUE;
        CmdMoveVelocity.Velocity := ApproachVelocity;
        CmdMoveVelocity.Acceleration := Acceleration;
        CmdMoveVelocity.Deceleration := Deceleration;
        CmdMoveVelocity.Direction := -1;

        InMotion := StaMoveVelocity.Busy;

        IF OvertravelNegActive THEN
            CmdMoveVelocity.Execute := FALSE;
            eState := HL_DETECT;

        ELSIF StaMoveVelocity.Error THEN
            CmdMoveVelocity.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_DETECT THEN

    (* HL_DETECT - Stop motion after switch detected *)
        CmdStop.Deceleration := Deceleration;
        CmdStop.Execute := TRUE;

        InMotion := NOT StaStop.Done;

        IF StaStop.Done THEN
            CmdStop.Execute := FALSE;
            eState := HL_BACKOFF;

        ELSIF StaStop.Error THEN
            CmdStop.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_BACKOFF THEN

    (* HL_BACKOFF - Move positive at slow speed until switch clears *)
        CmdMoveVelocity.Execute := TRUE;
        CmdMoveVelocity.Velocity := BackoffVelocity;
        CmdMoveVelocity.Acceleration := Acceleration;
        CmdMoveVelocity.Deceleration := Deceleration;
        CmdMoveVelocity.Direction := 1;

        InMotion := StaMoveVelocity.Busy;

        IF NOT OvertravelNegActive THEN
            CmdMoveVelocity.Execute := FALSE;
            bStopDoneLatched := FALSE;
            eState := HL_SETREF;

        ELSIF StaMoveVelocity.Error THEN
            CmdMoveVelocity.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_SETREF THEN

    (* HL_SETREF - Stop, then set position reference *)
        IF NOT bStopDoneLatched THEN
            CmdStop.Deceleration := Deceleration;
            CmdStop.Execute := NOT StaStop.Done;

            IF StaStop.Done THEN
                CmdStop.Execute := FALSE;
                bStopDoneLatched := TRUE;
            END_IF;
        ELSE
            CmdSetPosition.Execute := TRUE;
            CmdSetPosition.Position := SetPosition;
            CmdSetPosition.Mode := FALSE;

            IF StaSetPosition.Done THEN
                CmdSetPosition.Execute := FALSE;
                InMotion := FALSE;
                Done := TRUE;
                eState := HL_DONE;

            ELSIF StaSetPosition.Error THEN
                CmdSetPosition.Execute := FALSE;
                Error := TRUE;
                eState := HL_ERROR;
            END_IF;
        END_IF;


ELSIF eState = HL_DONE THEN

    (* HL_DONE - Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;

        IF NOT Execute THEN
            Done := FALSE;
            eState := HL_IDLE;
        END_IF;


ELSIF eState = HL_ABORTING THEN

    (* HL_ABORTING - Stop motion *)
        CmdStop.Deceleration := Deceleration;
        CmdStop.Execute := TRUE;

        IF StaStop.Done THEN
            CmdStop.Execute := FALSE;
            Active := FALSE;
            InMotion := FALSE;
            eState := HL_IDLE;

        ELSIF StaStop.Error THEN
            CmdStop.Execute := FALSE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_ERROR THEN

    (* HL_ERROR *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        IF NOT Execute THEN
            Error := FALSE;
            eState := HL_IDLE;
        END_IF;

    END_IF;  (* state machine *)
END_IF;  (* Enable gate *)

(* --------------------------------------------------------------------------
   Timer calls - exactly one site per instance per scan
   -------------------------------------------------------------------------- *)
tTimeout(IN := bTimeoutIN, PT := Timeout);
