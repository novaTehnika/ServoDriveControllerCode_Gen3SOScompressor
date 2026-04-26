(* ============================================================================
   FB_HomeLimit - Home-to-Negative-Overtravel Sequence Handler (Mode 110)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Drives the ram in the negative direction until the negative overtravel
   switch is tripped, backs off until the switch clears, then sets the
   position reference. This establishes the single coordinate frame used
   by the rest of the system.

   Uses the PLCopen Toolbox Jog FB (level-sensitive Forward/Reverse) in
   place of raw MC_MoveVelocity + MC_Stop. Jog handles deceleration
   internally when Forward/Reverse go FALSE; the explicit DETECT state
   (MC_Stop wait) is replaced by waiting on Jog.Done.

   Sequence:
   1. APPROACH:     Jog Reverse=TRUE at ApproachVelocity toward switch.
   2. AWAIT_STOP_1: Jog Reverse cleared on OvertravelNegActive; wait for Jog.Done.
   3. BACKOFF:      Jog Forward=TRUE at BackoffVelocity until switch clears.
   4. AWAIT_STOP_2: Jog Forward cleared; wait for Jog.Done.
   5. SETREF:       MC_SetPosition(SetPosition) to establish coordinate frame.

   Error conditions:
   - OvertravelPosActive at any point during the sequence.
   - Jog reporting Error in any moving state.
   - Overall Timeout expiring.

   Integration:
   - Called once per scan at end of PRG_Main.
   - CmdJog routed to G_cmdJog by PRG_Main when ST_HOME_LIMIT is active.
   - CmdSetPosition routed to G_cmdSetPosition by PRG_Main.
   - PRG_Main passes G_staJog into StaJog and G_staSetPosition into StaSetPosition.

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
       StaJog              : ST_StaJog              (* Status from Jog FB
       StaSetPosition      : ST_StaSetPosition      (* Status from MC_SetPosition
     VAR_OUTPUT:
       Active              : BOOL                   (* Sequence is running
       Done                : BOOL                   (* Homing complete
       InMotion            : BOOL                   (* Currently moving
       Error               : BOOL                   (* Error occurred
       CmdJog              : ST_CmdJog              (* Command output for Jog FB
       CmdSetPosition      : ST_CmdSetPosition      (* Command output for MC_SetPosition
     VAR:
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       tTimeout            : TON              (* Single call site - see end of body
       bTimeoutIN          : BOOL             (* Latched IN for tTimeout
     VAR CONSTANT:
       HL_IDLE             : INT := 0
       HL_APPROACH         : INT := 1
       HL_AWAIT_STOP_1     : INT := 2
       HL_BACKOFF          : INT := 3
       HL_AWAIT_STOP_2     : INT := 4
       HL_SETREF           : INT := 5
       HL_DONE             : INT := 6
       HL_ABORTING         : INT := 7
       HL_ERROR            : INT := 8
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
    CmdJog.Forward := FALSE;
    CmdJog.Reverse := FALSE;
    CmdJog.Velocity := LREAL#0.0;
    CmdSetPosition.Execute := FALSE;
    bTimeoutIN := FALSE;

ELSE
    (* Detect rising edge on Execute *)
    rTrigExecute(CLK := Execute);

    (* Handle abort in most states *)
    IF Abort AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND (eState <> HL_DONE) THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdSetPosition.Execute := FALSE;
        eState := HL_ABORTING;
    END_IF;

    (* Positive overtravel must never trigger during homing. *)
    IF OvertravelPosActive AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND
       (eState <> HL_DONE) AND (eState <> HL_ABORTING) THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdSetPosition.Execute := FALSE;
        Error := TRUE;
        eState := HL_ERROR;
    END_IF;

    (* Default timer IN; overall timeout check uses last-scan Q *)
    bTimeoutIN := (eState <> HL_IDLE) AND (eState <> HL_DONE) AND (eState <> HL_ERROR) AND (eState <> HL_ABORTING);

    IF tTimeout.Q THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
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

        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdSetPosition.Execute := FALSE;

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := HL_APPROACH;
        END_IF;


ELSIF eState = HL_APPROACH THEN

    (* HL_APPROACH - Jog negative toward overtravel / home switch *)
        CmdJog.Reverse := TRUE;
        CmdJog.Forward := FALSE;
        CmdJog.Velocity := ApproachVelocity;
        CmdJog.Acceleration := Acceleration;
        CmdJog.Deceleration := Deceleration;

        InMotion := StaJog.Busy;

        IF OvertravelNegActive THEN
            CmdJog.Reverse := FALSE;       (* Jog auto-decels *)
            eState := HL_AWAIT_STOP_1;

        ELSIF StaJog.Error THEN
            CmdJog.Reverse := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_AWAIT_STOP_1 THEN

    (* HL_AWAIT_STOP_1 - Wait for Jog to finish decelerating after switch detection *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done THEN
            eState := HL_BACKOFF;

        ELSIF StaJog.Error THEN
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_BACKOFF THEN

    (* HL_BACKOFF - Jog positive until switch clears *)
        CmdJog.Forward := TRUE;
        CmdJog.Reverse := FALSE;
        CmdJog.Velocity := BackoffVelocity;
        CmdJog.Acceleration := Acceleration;
        CmdJog.Deceleration := Deceleration;

        InMotion := StaJog.Busy;

        IF NOT OvertravelNegActive THEN
            CmdJog.Forward := FALSE;       (* Jog auto-decels *)
            eState := HL_AWAIT_STOP_2;

        ELSIF StaJog.Error THEN
            CmdJog.Forward := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_AWAIT_STOP_2 THEN

    (* HL_AWAIT_STOP_2 - Wait for Jog to finish decelerating after switch clears *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done THEN
            eState := HL_SETREF;

        ELSIF StaJog.Error THEN
            Error := TRUE;
            eState := HL_ERROR;
        END_IF;


ELSIF eState = HL_SETREF THEN

    (* HL_SETREF - Set position reference to establish coordinate frame *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := FALSE;

        CmdSetPosition.Execute := TRUE;
        CmdSetPosition.Position := SetPosition;
        CmdSetPosition.Mode := FALSE;

        IF StaSetPosition.Done THEN
            CmdSetPosition.Execute := FALSE;
            Done := TRUE;
            eState := HL_DONE;

        ELSIF StaSetPosition.Error THEN
            CmdSetPosition.Execute := FALSE;
            Error := TRUE;
            eState := HL_ERROR;
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

    (* HL_ABORTING - Jog inputs already cleared; wait for decel to finish *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done OR NOT StaJog.Busy THEN
            Active := FALSE;
            InMotion := FALSE;
            eState := HL_IDLE;

        ELSIF StaJog.Error THEN
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
