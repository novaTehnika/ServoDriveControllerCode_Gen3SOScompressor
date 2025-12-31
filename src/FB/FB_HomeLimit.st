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
   2. DETECT: Halt motion when home limit switch detected
   3. BACKOFF: Move away from switch at slow speed until switch clears
   4. SETREF: Set position reference at switch edge location

   Over-Travel Handling:
   - If retract limit triggers during approach from negative direction,
     the FB automatically reverses to approach from positive direction
   - If retract limit triggers during positive approach, this is a fault

   Integration:
   - Called from ST_HOME_LIMIT state in PRG_Main
   - Abort input for motion enable drop handling
   ============================================================================ *)

FUNCTION_BLOCK FB_HomeLimit
VAR_INPUT
    Enable              : BOOL;             (* Enable the function block *)
    Execute             : BOOL;             (* Start homing sequence *)
    Axis                : AXIS_REF;         (* PLCopen axis reference *)

    (* Configuration *)
    ApproachVelocity    : LREAL;            (* Approach velocity (mm/s) *)
    BackoffVelocity     : LREAL;            (* Backoff velocity (mm/s) *)
    SetPosition         : LREAL;            (* Position to set at home switch (mm) *)
    Timeout             : TIME;             (* Overall sequence timeout *)

    (* Limit switch inputs *)
    LimitHomeActive     : BOOL;             (* Home limit switch is triggered *)
    LimitRetractActive  : BOOL;             (* Retract limit switch is triggered *)

    (* Control *)
    Abort               : BOOL;             (* Abort the sequence *)
END_VAR

VAR_OUTPUT
    Active              : BOOL;             (* Sequence is running *)
    Done                : BOOL;             (* Homing complete *)
    InMotion            : BOOL;             (* Currently moving *)
    Error               : BOOL;             (* Error occurred *)
    ErrorID             : DWORD;            (* Error code if Error=TRUE *)
    ApproachingPositive : BOOL;             (* Currently approaching from positive direction *)
END_VAR

VAR
    (* PLCopen function blocks *)
    fbMoveVelocity      : MC_MoveVelocity;
    fbHalt              : MC_Halt;
    fbSetPosition       : MC_SetPosition;

    (* State machine *)
    eState              : INT := HL_IDLE;

    (* Edge detection *)
    rTrigExecute        : R_TRIG;

    (* Timers *)
    tTimeout            : TON;

    (* Internal flags *)
    bApproachFromPositive : BOOL := FALSE;  (* Direction flag for over-travel handling *)
END_VAR

(* ============================================================================
   Constants
   ============================================================================ *)
VAR CONSTANT
    HL_IDLE             : INT := 0;         (* Idle state *)
    HL_APPROACH         : INT := 1;         (* Approaching limit switch *)
    HL_DETECT           : INT := 2;         (* Halt after limit detected *)
    HL_BACKOFF          : INT := 3;         (* Backing off from switch *)
    HL_SETREF           : INT := 4;         (* Setting position reference *)
    HL_DONE             : INT := 5;         (* Sequence complete *)
    HL_ABORTING         : INT := 6;         (* Aborting sequence *)
    HL_ERROR            : INT := 7;         (* Error state *)

    (* Error codes *)
    ERR_TIMEOUT         : DWORD := 16#8001; (* Homing timeout *)
    ERR_DRIVE           : DWORD := 16#8002; (* Drive error during homing *)
    ERR_LIMIT_INVALID   : DWORD := 16#8003; (* Invalid limit switch sequence *)
    ERR_SETPOS          : DWORD := 16#8004; (* Set position failed *)
END_VAR

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
    fbHalt(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    tTimeout(IN := FALSE);
    RETURN;
END_IF

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in most states *)
IF Abort AND (eState <> HL_IDLE) AND (eState <> HL_ERROR) AND (eState <> HL_DONE) THEN
    fbMoveVelocity(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    tTimeout(IN := FALSE);
    eState := HL_ABORTING;
END_IF

(* Overall timeout check *)
tTimeout(IN := (eState <> HL_IDLE) AND (eState <> HL_DONE) AND (eState <> HL_ERROR) AND (eState <> HL_ABORTING),
         PT := Timeout);

IF tTimeout.Q THEN
    fbMoveVelocity(Execute := FALSE);
    fbHalt(Execute := FALSE);
    fbSetPosition(Execute := FALSE);
    Error := TRUE;
    ErrorID := ERR_TIMEOUT;
    eState := HL_ERROR;
END_IF

CASE eState OF

    HL_IDLE:
        (* Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        InMotion := FALSE;
        Error := FALSE;
        ErrorID := 0;
        ApproachingPositive := FALSE;
        bApproachFromPositive := FALSE;

        (* Reset function blocks *)
        fbMoveVelocity(Execute := FALSE);
        fbHalt(Execute := FALSE);
        fbSetPosition(Execute := FALSE);
        tTimeout(IN := FALSE);

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            bApproachFromPositive := FALSE;  (* Start approaching from negative *)
            eState := HL_APPROACH;
        END_IF


    HL_APPROACH:
        (* Move toward home limit switch
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
        END_IF

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
            END_IF

        ELSIF fbMoveVelocity.Error THEN
            fbMoveVelocity(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF


    HL_DETECT:
        (* Halt motion after limit switch detected *)
        fbHalt(
            Axis := Axis,
            Execute := TRUE
        );

        InMotion := NOT fbHalt.Done;

        IF fbHalt.Done THEN
            fbHalt(Execute := FALSE);
            eState := HL_BACKOFF;

        ELSIF fbHalt.Error THEN
            fbHalt(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF


    HL_BACKOFF:
        (* Move away from limit switch at slow speed until switch clears *)
        fbMoveVelocity(
            Axis := Axis,
            Execute := TRUE,
            Velocity := BackoffVelocity,
            Direction := MC_Positive_Direction
        );

        InMotion := fbMoveVelocity.Busy;

        IF NOT LimitHomeActive THEN
            (* Switch cleared - halt and set position *)
            fbMoveVelocity(Execute := FALSE);
            eState := HL_SETREF;

        ELSIF fbMoveVelocity.Error THEN
            fbMoveVelocity(Execute := FALSE);
            Error := TRUE;
            ErrorID := ERR_DRIVE;
            eState := HL_ERROR;
        END_IF


    HL_SETREF:
        (* Set position reference at home switch edge location *)
        (* First halt to ensure fully stopped *)
        fbHalt(
            Axis := Axis,
            Execute := NOT fbHalt.Done
        );

        IF fbHalt.Done THEN
            fbHalt(Execute := FALSE);

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
            END_IF
        END_IF


    HL_DONE:
        (* Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;
        ApproachingPositive := FALSE;

        (* Reset when Execute goes low *)
        IF NOT Execute THEN
            Done := FALSE;
            eState := HL_IDLE;
        END_IF


    HL_ABORTING:
        (* Aborting sequence - halt motion *)
        fbHalt(
            Axis := Axis,
            Execute := TRUE
        );

        IF fbHalt.Done THEN
            fbHalt(Execute := FALSE);
            Active := FALSE;
            InMotion := FALSE;
            eState := HL_IDLE;

        ELSIF fbHalt.Error THEN
            (* Halt failed - still try to go to idle *)
            fbHalt(Execute := FALSE);
            eState := HL_ERROR;
        END_IF


    HL_ERROR:
        (* Error state *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        (* Allow reset when Execute goes low *)
        IF NOT Execute THEN
            Error := FALSE;
            ErrorID := 0;
            eState := HL_IDLE;
        END_IF

END_CASE

END_FUNCTION_BLOCK
