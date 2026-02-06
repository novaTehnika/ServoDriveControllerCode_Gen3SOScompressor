(* ============================================================================
   FB_HandshakeManager - Master-Slave Handshake Manager

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Manages the handshake protocol between master and slave for:
   1. Mode entry handshake: 3-bit mode command/confirmation with timeout
   2. Fault reset handshake: FaultReset HIGH + MotionEnable LOW + fault mirroring

   Mode Entry Handshake Sequence:
   1. Master sets mode bits (DI0-2)
   2. Slave decodes and validates mode
   3. Slave sets confirmation bits (DO0-2) to match
   4. Master sets diMotionEnable HIGH
   5. Slave verifies mode bits still match -> handshake complete
   6. If timeout or mismatch -> handshake failed

   Fault Reset Handshake (with fault code mirroring):
   1. Slave is in fault state with doFaultActive = HIGH
   2. Slave outputs fault code on DO0-DO2
   3. Master reads fault code and sets DI0-DI2 to mirror it
   4. Master sets diFaultReset HIGH while diMotionEnable = LOW
   5. Slave verifies DI0-DI2 matches fault code (level-based, not edge)
   6. FaultResetValid goes TRUE, caller clears fault
   7. Master sees fault cleared, releases diFaultReset

   NOTE: All inputs should be pre-filtered/debounced by the caller.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable handshake manager
       RequestedMode       : E_OperatingMode  (* Mode requested by master
       ModeStable          : BOOL             (* Mode inputs are stable
       MotionEnable        : BOOL             (* Motion enable from master - should be filtered
       FaultReset          : BOOL             (* Fault reset signal from master - should be filtered
       InFaultState        : BOOL             (* Currently in fault state
       ActiveFaultCode     : E_FaultCode      (* Active fault code for mirroring check
       HandshakeTimeout    : TIME := T#500MS  (* Timeout for handshake
     VAR_OUTPUT:
       ConfirmedMode       : E_OperatingMode  (* Current confirmed mode (valid when HandshakeComplete)
       HandshakeActive     : BOOL             (* TRUE from handshake start until complete/timeout
       HandshakeComplete   : BOOL             (* TRUE when handshake succeeded
       HandshakeTimeout_Q  : BOOL             (* TRUE when timeout occurred
       ConfirmBit0         : BOOL             (* DO0 value
       ConfirmBit1         : BOOL             (* DO1 value
       ConfirmBit2         : BOOL             (* DO2 value
       FaultResetEdge      : BOOL             (* Fault reset rising edge detected (single scan pulse)
       FaultResetValid     : BOOL             (* Fault reset conditions met (level-based)
     VAR:
       eState              : INT := 0         (* Handshake state - initialized to HS_IDLE
       tTimeout            : TON              (* Handshake timeout timer
       rTrigFaultReset     : R_TRIG           (* Rising edge on fault reset
       eModeAtStart        : E_OperatingMode  (* Mode when handshake started
       bModeStableAtStart  : BOOL             (* Capture mode stability at handshake start
     VAR CONSTANT:
       HS_IDLE             : INT := 0         (* Idle, no handshake active
       HS_WAIT_MOTION_EN   : INT := 1         (* Waiting for motion enable
       HS_VERIFY_MODE      : INT := 2         (* Verifying mode match
       HS_COMPLETE         : INT := 3         (* Handshake complete
       HS_TIMEOUT          : INT := 4         (* Handshake timed out
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    ConfirmedMode := MODE_IDLE;
    HandshakeActive := FALSE;
    HandshakeComplete := FALSE;
    HandshakeTimeout_Q := FALSE;
    ConfirmBit0 := FALSE;
    ConfirmBit1 := FALSE;
    ConfirmBit2 := FALSE;
    FaultResetEdge := FALSE;
    FaultResetValid := FALSE;
    eState := HS_IDLE;
    tTimeout(IN := FALSE);
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Fault Reset Handshake Detection (with fault code mirroring)
   Uses LEVEL-based detection to avoid race conditions.
   The caller should detect the rising edge of FaultResetValid if needed.
   -------------------------------------------------------------------------- *)
rTrigFaultReset(CLK := FaultReset);
FaultResetEdge := rTrigFaultReset.Q;

(* Fault reset is valid when ALL conditions are met (level-based):
   - FaultReset is HIGH (level, not edge)
   - MotionEnable is LOW
   - We are in fault state
   - Active fault code is not FAULT_NONE
   - Master has mirrored the fault code on DI0-2 (RequestedMode matches ActiveFaultCode) *)
FaultResetValid := FaultReset AND
                   (NOT MotionEnable) AND
                   InFaultState AND
                   (ActiveFaultCode <> FAULT_NONE) AND
                   FaultMatchesModeBits(ActiveFaultCode, RequestedMode);

(* --------------------------------------------------------------------------
   Mode Handshake State Machine
   -------------------------------------------------------------------------- *)
CASE eState OF

    HS_IDLE:
        (* Reset outputs *)
        HandshakeActive := FALSE;
        HandshakeComplete := FALSE;
        HandshakeTimeout_Q := FALSE;
        tTimeout(IN := FALSE);

        (* Start handshake when mode is stable and different from idle
           (or when transitioning to a new mode) *)
        IF ModeStable AND (RequestedMode <> MODE_IDLE) AND NOT MotionEnable THEN
            (* Set confirmation to match requested mode *)
            ConfirmedMode := RequestedMode;
            ModeToBits(RequestedMode, ConfirmBit0, ConfirmBit1, ConfirmBit2);
            eModeAtStart := RequestedMode;
            bModeStableAtStart := TRUE;
            HandshakeActive := TRUE;
            eState := HS_WAIT_MOTION_EN;
            tTimeout(IN := TRUE, PT := HandshakeTimeout);
        ELSIF ModeStable AND (RequestedMode = MODE_IDLE) THEN
            (* Idle mode - just confirm it *)
            ConfirmedMode := MODE_IDLE;
            ModeToBits(MODE_IDLE, ConfirmBit0, ConfirmBit1, ConfirmBit2);
        END_IF;

    HS_WAIT_MOTION_EN:
        (* Waiting for master to assert motion enable *)
        (* Timer must be called each scan to update *)
        tTimeout(IN := TRUE, PT := HandshakeTimeout);

        IF tTimeout.Q THEN
            (* Timeout - handshake failed *)
            tTimeout(IN := FALSE);
            eState := HS_TIMEOUT;

        ELSIF RequestedMode <> eModeAtStart THEN
            (* Mode changed during handshake - abort and reset timer *)
            tTimeout(IN := FALSE);
            eState := HS_IDLE;

        ELSIF MotionEnable THEN
            (* Motion enable received, verify mode still matches *)
            eState := HS_VERIFY_MODE;
        END_IF;

    HS_VERIFY_MODE:
        (* Verify requested mode hasn't changed since handshake start *)
        IF RequestedMode = eModeAtStart THEN
            (* Mode matches - handshake complete *)
            HandshakeComplete := TRUE;
            HandshakeActive := FALSE;
            tTimeout(IN := FALSE);
            eState := HS_COMPLETE;
        ELSE
            (* Mismatch - treat as timeout/failure *)
            tTimeout(IN := FALSE);
            eState := HS_TIMEOUT;
        END_IF;

    HS_COMPLETE:
        (* Handshake complete - stay here until motion enable drops *)
        IF NOT MotionEnable THEN
            (* Motion disabled - return to idle *)
            HandshakeComplete := FALSE;
            eState := HS_IDLE;
        END_IF;

    HS_TIMEOUT:
        (* Handshake timed out or failed *)
        HandshakeTimeout_Q := TRUE;
        HandshakeActive := FALSE;
        ConfirmedMode := MODE_IDLE;  (* Reset confirmed mode on failure *)
        ModeToBits(MODE_IDLE, ConfirmBit0, ConfirmBit1, ConfirmBit2);
        tTimeout(IN := FALSE);

        (* Exit timeout when motion enable drops - simpler and more reliable *)
        IF NOT MotionEnable THEN
            HandshakeTimeout_Q := FALSE;
            eState := HS_IDLE;
        END_IF;

END_CASE;
