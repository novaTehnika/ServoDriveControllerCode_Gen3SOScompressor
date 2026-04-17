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
   - Called once per scan at end of PRG_Main
   - CmdMoveVelocity/CmdStop/CmdEncMngr routed to G_cmd* by PRG_Main
   - Reads G_staMoveVelocity/G_staStop/G_staEncMngr for status

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       Execute             : BOOL             (* Start homing sequence
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
       ApproachingPositive : BOOL             (* Currently approaching from positive direction
       CmdMoveVelocity     : ST_CmdMoveVelocity  (* Command output for MC_MoveVelocity
       CmdStop             : ST_CmdStop           (* Command output for MC_Stop
       CmdEncMngr          : ST_CmdEncMngr         (* Command output for AbsolutePositionManager
     VAR:
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       tTimeout            : TON
       bApproachFromPositive : BOOL := FALSE
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
    ApproachingPositive := FALSE;
    eState := HL_IDLE;
    bApproachFromPositive := FALSE;
    bStopDoneLatched := FALSE;
    CmdMoveVelocity.Execute := FALSE;
    CmdStop.Execute := FALSE;
    CmdEncMngr.SetPosition := FALSE;
    tTimeout(IN := FALSE);
    RETURN;
END_IF;

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in most states *)
IF Abort AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND (eState <> HL_DONE) THEN
    CmdMoveVelocity.Execute := FALSE;
    CmdEncMngr.SetPosition := FALSE;
    tTimeout(IN := FALSE);
    eState := HL_ABORTING;
END_IF;

(* Overall timeout check *)
tTimeout(IN := (eState <> HL_IDLE) AND (eState <> HL_DONE) AND (eState <> HL_ERROR) AND (eState <> HL_ABORTING),
         PT := Timeout);

IF tTimeout.Q THEN
    CmdMoveVelocity.Execute := FALSE;
    CmdStop.Execute := FALSE;
    CmdEncMngr.SetPosition := FALSE;
    Error := TRUE;
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
        ApproachingPositive := FALSE;
        bApproachFromPositive := FALSE;
        bStopDoneLatched := FALSE;

        CmdMoveVelocity.Execute := FALSE;
        CmdStop.Execute := FALSE;
        CmdEncMngr.SetPosition := FALSE;
        tTimeout(IN := FALSE);

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            bApproachFromPositive := FALSE;
            eState := HL_APPROACH;
        END_IF;


ELSIF eState = HL_APPROACH THEN

    (* HL_APPROACH - Move toward home limit switch *)
        ApproachingPositive := bApproachFromPositive;

        CmdMoveVelocity.Execute := TRUE;
        CmdMoveVelocity.Velocity := ApproachVelocity;
        CmdMoveVelocity.Acceleration := LREAL#0.0;
        CmdMoveVelocity.Deceleration := LREAL#0.0;
        IF bApproachFromPositive THEN
            CmdMoveVelocity.Direction := 1;
        ELSE
            CmdMoveVelocity.Direction := -1;
        END_IF;

        InMotion := G_staMoveVelocity.Busy;

        IF LimitHomeActive THEN
            CmdMoveVelocity.Execute := FALSE;
            eState := HL_DETECT;

        ELSIF LimitRetractActive THEN
            IF NOT bApproachFromPositive THEN
                CmdMoveVelocity.Execute := FALSE;
                bApproachFromPositive := TRUE;

            ELSE
                CmdMoveVelocity.Execute := FALSE;
                Error := TRUE;
                eState := HL_ERROR;
            END_IF;

        ELSIF G_staMoveVelocity.Error THEN
            CmdMoveVelocity.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_DETECT THEN

    (* HL_DETECT - Stop motion after limit switch detected *)
        CmdStop.Execute := TRUE;

        InMotion := NOT G_staStop.Done;

        IF G_staStop.Done THEN
            CmdStop.Execute := FALSE;
            eState := HL_BACKOFF;

        ELSIF G_staStop.Error THEN
            CmdStop.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_BACKOFF THEN

    (* HL_BACKOFF - Move away from limit switch at slow speed until switch clears *)
        CmdMoveVelocity.Execute := TRUE;
        CmdMoveVelocity.Velocity := BackoffVelocity;
        CmdMoveVelocity.Acceleration := LREAL#0.0;
        CmdMoveVelocity.Deceleration := LREAL#0.0;
        CmdMoveVelocity.Direction := 1;

        InMotion := G_staMoveVelocity.Busy;

        IF NOT LimitHomeActive THEN
            CmdMoveVelocity.Execute := FALSE;
            bStopDoneLatched := FALSE;
            eState := HL_SETREF;

        ELSIF G_staMoveVelocity.Error THEN
            CmdMoveVelocity.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_SETREF THEN

    (* HL_SETREF - Set position reference at home switch edge location *)
        IF NOT bStopDoneLatched THEN
            CmdStop.Execute := NOT G_staStop.Done;

            IF G_staStop.Done THEN
                CmdStop.Execute := FALSE;
                bStopDoneLatched := TRUE;
            END_IF;
        ELSE
            CmdEncMngr.Enable := TRUE;
            CmdEncMngr.SetPosition := TRUE;
            CmdEncMngr.Position := SetPosition;

            IF G_staEncMngr.SetPositionDone THEN
                CmdEncMngr.SetPosition := FALSE;
                InMotion := FALSE;
                Done := TRUE;
                eState := HL_DONE;

            ELSIF G_staEncMngr.Error THEN
                CmdEncMngr.SetPosition := FALSE;
                Error := TRUE;
                eState := HL_ERROR;
            END_IF;
        END_IF;


ELSIF eState = HL_DONE THEN

    (* HL_DONE - Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;
        ApproachingPositive := FALSE;

        IF NOT Execute THEN
            Done := FALSE;
            eState := HL_IDLE;
        END_IF;


ELSIF eState = HL_ABORTING THEN

    (* HL_ABORTING - Stop motion *)
        CmdStop.Execute := TRUE;

        IF G_staStop.Done THEN
            CmdStop.Execute := FALSE;
            Active := FALSE;
            InMotion := FALSE;
            eState := HL_IDLE;

        ELSIF G_staStop.Error THEN
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

END_IF;
