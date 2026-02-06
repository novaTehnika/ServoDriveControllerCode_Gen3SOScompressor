(* ============================================================================
   FB_EncoderManager - Absolute Encoder Validation Manager

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Validates absolute encoder status on startup using Yaskawa's
   AbsolutePositionManager function block. Determines if homing is required
   due to encoder battery failure or multi-turn data corruption.

   The Sigma-7 absolute encoder maintains position across power cycles via
   battery backup. If the battery fails or multi-turn data is corrupted,
   the encoder position is invalid and homing is required.

   States:
   - IDLE: Waiting for check trigger
   - CHECK_INIT: Initialize AbsolutePositionManager
   - CHECK_STATUS: Query encoder status
   - COMPLETE: Check finished, outputs valid
   - ERROR: Error during check

   Integration:
   - Called from ST_ENCODER_CHECK state in PRG_Main
   - Sets AbsHomeRequired output if encoder invalid
   - PRG_Main uses this to set flagAbsHomeRequired

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       Axis                : AXIS_REF         (* PLCopen axis reference
       CheckOnStartup      : BOOL             (* Trigger startup check
     VAR_OUTPUT:
       EncoderValid        : BOOL             (* Absolute position is valid
       BatteryOK           : BOOL             (* Encoder battery status OK
       MultiTurnValid      : BOOL             (* Multi-turn counter data valid
       AbsHomeRequired     : BOOL             (* Homing required due to encoder issue
       CheckComplete       : BOOL             (* Startup check finished
       Busy                : BOOL             (* Check in progress
       Error               : BOOL             (* Error occurred during check
       ErrorID             : DWORD            (* Error code if Error=TRUE
     VAR:
       fbAbsPosManager     : AbsolutePositionManager
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
    (* Reset all outputs when disabled *)
    EncoderValid := FALSE;
    BatteryOK := FALSE;
    MultiTurnValid := FALSE;
    AbsHomeRequired := FALSE;
    CheckComplete := FALSE;
    Busy := FALSE;
    Error := FALSE;
    ErrorID := 0;
    eState := ENC_IDLE;
    tCheckTimeout(IN := FALSE);
    fbAbsPosManager(Enable := FALSE);
    RETURN;
END_IF;

(* Detect rising edge on CheckOnStartup *)
rTrigCheck(CLK := CheckOnStartup);

CASE eState OF

    ENC_IDLE:
        (* Waiting for check trigger *)
        Busy := FALSE;
        CheckComplete := FALSE;

        IF rTrigCheck.Q OR (CheckOnStartup AND NOT bCheckTriggered) THEN
            bCheckTriggered := TRUE;
            Busy := TRUE;
            Error := FALSE;
            ErrorID := 0;
            tCheckTimeout(IN := FALSE);
            eState := ENC_CHECK_INIT;
        END_IF;

    ENC_CHECK_INIT:
        (* Initialize AbsolutePositionManager *)
        (* Start timeout timer *)
        tCheckTimeout(IN := TRUE, PT := CHECK_TIMEOUT);

        (* Enable the AbsolutePositionManager *)
        fbAbsPosManager(
            Axis := Axis,
            Enable := TRUE
        );

        (* Wait for FB to initialize or check for immediate status *)
        IF fbAbsPosManager.Valid THEN
            eState := ENC_CHECK_STATUS;
        ELSIF fbAbsPosManager.Error THEN
            Error := TRUE;
            ErrorID := fbAbsPosManager.ErrorID;
            eState := ENC_ERROR;
        ELSIF tCheckTimeout.Q THEN
            Error := TRUE;
            ErrorID := 16#8001;  (* Timeout error code *)
            eState := ENC_ERROR;
        END_IF;

    ENC_CHECK_STATUS:
        (* Query encoder status from AbsolutePositionManager *)
        fbAbsPosManager(
            Axis := Axis,
            Enable := TRUE
        );

        (* Read status outputs *)
        IF fbAbsPosManager.Valid THEN
            (* Evaluate encoder health *)
            EncoderValid := fbAbsPosManager.AbsolutePositionValid;
            BatteryOK := NOT fbAbsPosManager.BatteryAlarm;
            MultiTurnValid := NOT fbAbsPosManager.MultiTurnOverflow;

            (* Homing required if any issue detected *)
            AbsHomeRequired := NOT EncoderValid OR
                               NOT BatteryOK OR
                               NOT MultiTurnValid;

            CheckComplete := TRUE;
            Busy := FALSE;
            tCheckTimeout(IN := FALSE);
            eState := ENC_COMPLETE;

        ELSIF fbAbsPosManager.Error THEN
            Error := TRUE;
            ErrorID := fbAbsPosManager.ErrorID;
            eState := ENC_ERROR;

        ELSIF tCheckTimeout.Q THEN
            Error := TRUE;
            ErrorID := 16#8001;  (* Timeout error code *)
            eState := ENC_ERROR;
        END_IF;

    ENC_COMPLETE:
        (* Check complete - hold outputs until disabled or new check *)
        Busy := FALSE;
        CheckComplete := TRUE;

        (* Allow re-check if CheckOnStartup toggled *)
        IF NOT CheckOnStartup THEN
            bCheckTriggered := FALSE;
        END_IF;

        IF rTrigCheck.Q THEN
            (* New check requested *)
            CheckComplete := FALSE;
            Busy := TRUE;
            tCheckTimeout(IN := FALSE);
            eState := ENC_CHECK_INIT;
        END_IF;

    ENC_ERROR:
        (* Error state *)
        Busy := FALSE;
        CheckComplete := FALSE;

        (* Assume encoder invalid on error - safe default *)
        EncoderValid := FALSE;
        BatteryOK := FALSE;
        MultiTurnValid := FALSE;
        AbsHomeRequired := TRUE;

        (* Disable the AbsolutePositionManager *)
        fbAbsPosManager(Enable := FALSE);
        tCheckTimeout(IN := FALSE);

        (* Allow retry if CheckOnStartup toggled *)
        IF NOT CheckOnStartup THEN
            bCheckTriggered := FALSE;
            Error := FALSE;
            ErrorID := 0;
            eState := ENC_IDLE;
        END_IF;

END_CASE;
