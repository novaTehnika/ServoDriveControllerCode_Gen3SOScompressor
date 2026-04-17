(* ============================================================================
   FB_EncoderManager - Absolute Encoder Validation Manager

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Validates absolute encoder status on startup by commanding the
   AbsolutePositionManager (running in the LD POU) via G_cmd/G_sta globals.
   Determines if homing is required due to encoder data corruption.

   The Sigma-7 absolute encoder maintains position across power cycles via
   battery backup. If the battery fails or multi-turn data is corrupted,
   the encoder position is invalid and homing is required.

   States:
   - IDLE: Waiting for check trigger
   - CHECK_INIT: Enable AbsolutePositionManager via G_cmdEncMngr
   - CHECK_STATUS: Query encoder status via G_staEncMngr
   - COMPLETE: Check finished, outputs valid
   - ERROR: Error during check

   Integration:
   - Called once per scan at end of PRG_Main
   - Commands AbsolutePositionManager via CmdEncMngr output
   - PRG_Main routes CmdEncMngr -> G_cmdEncMngr when active

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       CheckOnStartup      : BOOL             (* Trigger startup check
     VAR_OUTPUT:
       EncoderValid        : BOOL             (* Absolute position is valid
       AbsHomeRequired     : BOOL             (* Homing required due to encoder issue
       CheckComplete       : BOOL             (* Startup check finished
       Busy                : BOOL             (* Check in progress
       Error               : BOOL             (* Error occurred during check
       CmdEncMngr          : ST_CmdEncMngr    (* Command output for LD POU
     VAR:
       eState              : INT := 0         (* State machine
       rTrigCheck          : R_TRIG
       tCheckTimeout       : TON
       bCheckTriggered     : BOOL
     VAR CONSTANT:
       ENC_IDLE            : INT := 0
       ENC_CHECK_INIT      : INT := 1
       ENC_CHECK_STATUS    : INT := 2
       ENC_COMPLETE        : INT := 3
       ENC_ERROR           : INT := 4
       CHECK_TIMEOUT       : TIME := T#2S
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    EncoderValid := FALSE;
    AbsHomeRequired := FALSE;
    CheckComplete := FALSE;
    Busy := FALSE;
    Error := FALSE;
    eState := ENC_IDLE;
    tCheckTimeout(IN := FALSE);
    CmdEncMngr.Enable := FALSE;
    RETURN;
END_IF;

(* Detect rising edge on CheckOnStartup *)
rTrigCheck(CLK := CheckOnStartup);

(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

IF eState = ENC_IDLE THEN

    (* ENC_IDLE - Waiting for check trigger *)
        Busy := FALSE;
        CheckComplete := FALSE;
        CmdEncMngr.Enable := FALSE;

        IF rTrigCheck.Q OR (CheckOnStartup AND NOT bCheckTriggered) THEN
            bCheckTriggered := TRUE;
            Busy := TRUE;
            Error := FALSE;
            tCheckTimeout(IN := FALSE);
            eState := ENC_CHECK_INIT;
        END_IF;

ELSIF eState = ENC_CHECK_INIT THEN

    (* ENC_CHECK_INIT - Enable AbsolutePositionManager via command output *)
        tCheckTimeout(IN := TRUE, PT := CHECK_TIMEOUT);

        CmdEncMngr.Enable := TRUE;

        IF G_staEncMngr.Valid THEN
            eState := ENC_CHECK_STATUS;
        ELSIF G_staEncMngr.Error THEN
            Error := TRUE;
            eState := ENC_ERROR;
        ELSIF tCheckTimeout.Q THEN
            Error := TRUE;
            eState := ENC_ERROR;
        END_IF;

ELSIF eState = ENC_CHECK_STATUS THEN

    (* ENC_CHECK_STATUS - Query encoder status from LD POU *)
        CmdEncMngr.Enable := TRUE;

        IF G_staEncMngr.Valid THEN
            EncoderValid := G_staEncMngr.PositionValid;

            AbsHomeRequired := NOT EncoderValid;

            CheckComplete := TRUE;
            Busy := FALSE;
            tCheckTimeout(IN := FALSE);
            eState := ENC_COMPLETE;

        ELSIF G_staEncMngr.Error THEN
            Error := TRUE;
            eState := ENC_ERROR;

        ELSIF tCheckTimeout.Q THEN
            Error := TRUE;
            eState := ENC_ERROR;
        END_IF;

ELSIF eState = ENC_COMPLETE THEN

    (* ENC_COMPLETE - Check complete - hold outputs until disabled or new check *)
        Busy := FALSE;
        CheckComplete := TRUE;
        CmdEncMngr.Enable := FALSE;

        IF NOT CheckOnStartup THEN
            bCheckTriggered := FALSE;
        END_IF;

        IF rTrigCheck.Q THEN
            CheckComplete := FALSE;
            Busy := TRUE;
            tCheckTimeout(IN := FALSE);
            eState := ENC_CHECK_INIT;
        END_IF;

ELSIF eState = ENC_ERROR THEN

    (* ENC_ERROR *)
        Busy := FALSE;
        CheckComplete := FALSE;

        EncoderValid := FALSE;
        AbsHomeRequired := TRUE;

        CmdEncMngr.Enable := FALSE;
        tCheckTimeout(IN := FALSE);

        IF NOT CheckOnStartup THEN
            bCheckTriggered := FALSE;
            Error := FALSE;
            eState := ENC_IDLE;
        END_IF;

END_IF;
