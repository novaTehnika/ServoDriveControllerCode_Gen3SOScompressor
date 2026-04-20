(* ============================================================================
   PRG_Main - Main Program with State Machine

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Main program containing the central state machine for servo control.
   Coordinates all function blocks, handles state transitions, and manages
   the handshake protocol with the master controller.

   All built-in motion FBs (MC_Power, MC_Stop, MC_Reset, MC_MoveAbsolute,
   MC_MoveVelocity, MC_ReadActual*, Y_DirectControl, MC_SetPosition)
   are instantiated in the LD POU and communicate
   via G_cmd*/G_sta* structured globals.

   Every user-defined FB is called exactly once at the end of this file.

   State Machine Groups:
   - Idle: ST_IDLE (first scan goes directly here; both homing flags forced TRUE)
   - Transition (Activation): ST_DRIVE_ENABLE, ST_BRAKE_RELEASE
   - Operational: ST_BRAKE_HOLD, ST_POSITION_CTRL, ST_VELOCITY_CTRL,
                  ST_TORQUE_CTRL, ST_GO_HOME, ST_HOME_*
   - Transition (Inter-Mode): ST_HOLD_POSITION
   - Transition (Deactivation): ST_BRAKE_ENGAGE, ST_DRIVE_DISABLE
   - Fault: ST_FAULT, ST_FAULT_IDLE

   Variables (define in MWiec GUI):
     VAR:
       (* Function block instances
       fbFilterModeBit0    : FB_DigitalInputFilter
       fbFilterModeBit1    : FB_DigitalInputFilter
       fbFilterModeBit2    : FB_DigitalInputFilter
       fbFilterMotionEnable: FB_DigitalInputFilter
       fbFilterLimitRetract: FB_DigitalInputFilter
       fbFilterLimitHome   : FB_DigitalInputFilter
       fbFilterFaultReset  : FB_DigitalInputFilter
       fbFilterAnalog      : FB_AnalogInputFilter
       fbModeDecoder       : FB_ModeDecoder
       fbHandshake         : FB_HandshakeManager
       fbFaultReset        : FB_FaultResetHandler
       fbOutputMux         : FB_OutputMux
       fbAnalogProc        : FB_AnalogProcessor
       fbPosOutput         : FB_PositionOutput
       fbSafetyMonitor     : FB_SafetyMonitor
       fbPistonExitGuard   : FB_PistonExitGuard
       fbGoHome            : FB_GoHome
       fbHomeLimit         : FB_HomeLimit
       fbHomeEOT           : FB_HomeEOT
       (* Timers
       tStateTimer         : TON
       tBrakeTimer         : TON
       tDriveReadyTimer    : TON
       tHandshakeTimer     : TON
       (* Edge triggers
       rTrigMotionEnable   : R_TRIG
       fTrigMotionEnable   : F_TRIG
       (* Local variables
       bFirstScan          : BOOL := TRUE
       bEnterState         : BOOL
       eTargetState        : E_SystemState
       eTargetMode         : E_OperatingMode
       stCurrentLimits     : ST_AxisLimits
       bFilteredModeBit0   : BOOL
       bFilteredModeBit1   : BOOL
       bFilteredModeBit2   : BOOL
       bFilteredMotionEnable : BOOL
       bFilteredLimitRetract : BOOL
       bFilteredLimitHome  : BOOL
       bFilteredFaultReset : BOOL
       nFilteredAnalog     : LREAL
       rAnalogFilterAlpha  : LREAL := 0.0
       bFaultResetInFaultState : BOOL
       eFaultResetActiveCode   : E_FaultCode
   ============================================================================ *)

(* ============================================================================
   MAIN PROGRAM BODY
   ============================================================================ *)

(* --------------------------------------------------------------------------
   First Scan Initialization
   -------------------------------------------------------------------------- *)
IF bFirstScan THEN
    bFirstScan := FALSE;

    G_sysCurrentState := E_SystemState#ST_IDLE;
    G_sysPreviousState := E_SystemState#ST_IDLE;

    G_flagEOTHomeRequired := TRUE;
    G_flagAbsHomeRequired := TRUE;

    G_doBrakeDisengage := FALSE;
    G_doPerformanceStatus := FALSE;
    G_doHomingComplete := FALSE;
    G_doInMotion := FALSE;

    IF G_cfgAnalogFilterTimeConst > T#0MS THEN
        rAnalogFilterAlpha := LREAL#0.001 / ((DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / LREAL#1000.0) + LREAL#0.001);
    ELSE
        rAnalogFilterAlpha := LREAL#1.0;
    END_IF;
    rAnalogFilterAlpha := LIMIT(LREAL#0.0, rAnalogFilterAlpha, LREAL#1.0);
END_IF;


(* --------------------------------------------------------------------------
   Input Processing
   Read filtered values from FB outputs (FBs called at end of file).
   Values reflect previous scan's filter outputs — negligible at 1ms cycle.
   -------------------------------------------------------------------------- *)

bFilteredModeBit0 := fbFilterModeBit0.FilteredOutput;
bFilteredModeBit1 := fbFilterModeBit1.FilteredOutput;
bFilteredModeBit2 := fbFilterModeBit2.FilteredOutput;
bFilteredMotionEnable := fbFilterMotionEnable.FilteredOutput;
bFilteredLimitRetract := fbFilterLimitRetract.FilteredOutput;
bFilteredLimitHome := fbFilterLimitHome.FilteredOutput;
bFilteredFaultReset := fbFilterFaultReset.FilteredOutput;
nFilteredAnalog := fbFilterAnalog.FilteredOutput;

G_sysRequestedMode := fbModeDecoder.DecodedMode;

rTrigMotionEnable(CLK := bFilteredMotionEnable);
fTrigMotionEnable(CLK := bFilteredMotionEnable);

G_sysLimitRetractActive := NOT bFilteredLimitRetract;
G_sysLimitHomeActive := NOT bFilteredLimitHome;


(* --------------------------------------------------------------------------
   State Machine
   No inline FB calls. Uses G_sta* for built-in FB status and
   previous-scan outputs for user FB status.
   -------------------------------------------------------------------------- *)

bEnterState := (G_sysCurrentState <> G_sysPreviousState);
IF bEnterState THEN
    IF G_sysCurrentState = E_SystemState#ST_FAULT THEN
        G_sysLastFaultState := G_sysPreviousState;
    END_IF;
    G_sysPreviousState := G_sysCurrentState;
    tStateTimer(IN := FALSE);
END_IF;

IF G_sysCurrentState = E_SystemState#ST_IDLE THEN

    (* ======================================================================
       IDLE STATE
       ====================================================================== *)

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            G_sysDriveEnabled := FALSE;
            G_sysDriveReady := FALSE;
            G_sysConfirmedMode := E_OperatingMode#MODE_IDLE;
        END_IF;

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF FN_IsOperationalMode(eTargetMode) THEN
                IF G_flagAbsHomeRequired OR G_flagEOTHomeRequired THEN
                    G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                    G_sysCurrentState := E_SystemState#ST_FAULT;
                ELSE
                    G_sysCurrentState := E_SystemState#ST_DRIVE_ENABLE;
                END_IF;

            ELSIF FN_IsHomingMode(eTargetMode) THEN
                IF (eTargetMode = E_OperatingMode#MODE_HOME_EOT) AND G_flagAbsHomeRequired THEN
                    G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                    G_sysCurrentState := E_SystemState#ST_FAULT;
                ELSE
                    G_sysCurrentState := E_SystemState#ST_DRIVE_ENABLE;
                END_IF;

            ELSE
                ;
            END_IF;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - ACTIVATION
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE THEN

        IF bEnterState THEN
            tDriveReadyTimer(IN := FALSE);
        END_IF;

        tDriveReadyTimer(IN := TRUE, PT := G_cfgDriveReadyTimeout);

        IF G_staPower.Status THEN
            G_sysDriveEnabled := TRUE;
            G_sysDriveReady := TRUE;
            G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;

        ELSIF G_staPower.Error OR tDriveReadyTimer.Q THEN
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE THEN

        IF bEnterState THEN
            G_doBrakeDisengage := TRUE;
            tBrakeTimer(IN := FALSE);
        END_IF;

        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeDisengageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := FALSE;

            IF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
            ELSIF eTargetMode = E_OperatingMode#MODE_POSITION THEN
                G_sysCurrentState := E_SystemState#ST_POSITION_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_VELOCITY THEN
                G_sysCurrentState := E_SystemState#ST_VELOCITY_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_TORQUE THEN
                G_sysCurrentState := E_SystemState#ST_TORQUE_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_GO_HOME THEN
                G_sysCurrentState := E_SystemState#ST_GO_HOME;
            ELSIF eTargetMode = E_OperatingMode#MODE_HOME_LIMIT THEN
                G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;
            ELSIF eTargetMode = E_OperatingMode#MODE_HOME_EOT THEN
                G_sysCurrentState := E_SystemState#ST_HOME_EOT;
            ELSE
                G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
            END_IF;
        END_IF;


    (* ======================================================================
       OPERATIONAL STATES
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD THEN

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            G_sysConfirmedMode := E_OperatingMode#MODE_BRAKE_HOLD;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        G_doInMotion := FALSE;

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF FN_IsOperationalMode(eTargetMode) AND
               (G_flagAbsHomeRequired OR G_flagEOTHomeRequired) THEN
                G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                G_sysCurrentState := E_SystemState#ST_FAULT;
            ELSE
                IF eTargetMode = E_OperatingMode#MODE_IDLE THEN
                    G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;

                ELSIF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                    ;

                ELSIF eTargetMode = E_OperatingMode#MODE_POSITION OR eTargetMode = E_OperatingMode#MODE_VELOCITY OR eTargetMode = E_OperatingMode#MODE_TORQUE OR
                      eTargetMode = E_OperatingMode#MODE_GO_HOME OR eTargetMode = E_OperatingMode#MODE_HOME_LIMIT OR eTargetMode = E_OperatingMode#MODE_HOME_EOT THEN
                    G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;
                END_IF;
            END_IF;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_POSITION_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_POSITION;
            stCurrentLimits.PosMax := G_cfgPosSoftLimitMax;
            stCurrentLimits.PosMin := G_cfgPosSoftLimitMin;
            stCurrentLimits.VelMax := G_cfgVelLimitNormal;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        G_sysCommandedPosition := fbAnalogProc.ScaledPosition - G_posEOTOffset;
        G_sysCommandedPosition := LIMIT(stCurrentLimits.PosMin,
                                      G_sysCommandedPosition,
                                      stCurrentLimits.PosMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_VELOCITY;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        G_sysCommandedVelocity := fbAnalogProc.ScaledVelocity;
        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      G_sysCommandedVelocity,
                                      stCurrentLimits.VelMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_TORQUE;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitMax;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
        END_IF;

        G_sysCommandedTorque := fbAnalogProc.ScaledTorque;
        G_sysCommandedTorque := LIMIT(-stCurrentLimits.TorqueMax,
                                    G_sysCommandedTorque,
                                    stCurrentLimits.TorqueMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_GO_HOME THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_GO_HOME;
        END_IF;

        G_doInMotion := fbGoHome.InMotion;

        IF fbGoHome.RedirectToHoming THEN
            G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;

        ELSIF fbGoHome.Done THEN
            G_doInMotion := FALSE;

        ELSIF fbGoHome.Error THEN
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIMIT THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_HOME_LIMIT;
            G_flagHomingInProgress := TRUE;
        END_IF;

        G_doInMotion := fbHomeLimit.InMotion;

        IF fbHomeLimit.Done THEN
            G_flagAbsHomeRequired := FALSE;
            G_flagHomingComplete := TRUE;
            G_sysCurrentState := E_SystemState#ST_HOME_COMPLETE;

        ELSIF fbHomeLimit.Error THEN
            G_sysActiveFault := E_FaultCode#FAULT_LIMIT_SWITCH;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIM_APPROACH OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_DETECT OR
      G_sysCurrentState = E_SystemState#ST_HOME_LIM_BACKOFF OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_SETREF THEN
        G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_EOT THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_HOME_EOT;
            G_flagHomingInProgress := TRUE;
        END_IF;

        G_doInMotion := fbHomeEOT.InMotion;

        IF fbHomeEOT.RequiresLimitHome THEN
            G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fbHomeEOT.Done THEN
            G_posActualEOT := fbHomeEOT.EOTPosition;
            G_posEOTOffset := fbHomeEOT.EOTOffset;
            G_flagEOTHomeRequired := FALSE;
            G_flagHomingComplete := TRUE;
            G_sysCurrentState := E_SystemState#ST_HOME_COMPLETE;

        ELSIF fbHomeEOT.Error THEN
            G_sysActiveFault := E_FaultCode#FAULT_POSITION;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
ELSIF G_sysCurrentState = E_SystemState#ST_HOME_EOT_FAST OR G_sysCurrentState = E_SystemState#ST_HOME_EOT_SLOW OR
      G_sysCurrentState = E_SystemState#ST_HOME_EOT_DETECT OR G_sysCurrentState = E_SystemState#ST_HOME_EOT_SETREF THEN
        G_sysCurrentState := E_SystemState#ST_HOME_EOT;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_COMPLETE THEN

        IF bEnterState THEN
            G_flagHomingInProgress := FALSE;
            G_doHomingComplete := TRUE;
        END_IF;

        G_doInMotion := FALSE;

        IF fTrigMotionEnable.Q THEN
            G_doHomingComplete := FALSE;
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* ======================================================================
       TRANSITION STATE - INTER-MODE
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_HOLD_POSITION THEN

        IF bEnterState THEN
            G_sysHalted := FALSE;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        IF NOT G_sysHalted THEN
            IF G_staStop.Done THEN
                G_sysHalted := TRUE;
            END_IF;
        END_IF;

        G_doInMotion := NOT G_sysHalted;

        tHandshakeTimer(IN := TRUE, PT := G_cfgHandshakeTimeout);

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF FN_IsOperationalMode(eTargetMode) AND
               (G_flagAbsHomeRequired OR G_flagEOTHomeRequired) THEN
                G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                G_sysCurrentState := E_SystemState#ST_FAULT;
            ELSE
                IF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                    G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
                ELSIF eTargetMode = E_OperatingMode#MODE_POSITION THEN
                    G_sysCurrentState := E_SystemState#ST_POSITION_CTRL;
                ELSIF eTargetMode = E_OperatingMode#MODE_VELOCITY THEN
                    G_sysCurrentState := E_SystemState#ST_VELOCITY_CTRL;
                ELSIF eTargetMode = E_OperatingMode#MODE_TORQUE THEN
                    G_sysCurrentState := E_SystemState#ST_TORQUE_CTRL;
                ELSIF eTargetMode = E_OperatingMode#MODE_GO_HOME THEN
                    G_sysCurrentState := E_SystemState#ST_GO_HOME;
                ELSIF eTargetMode = E_OperatingMode#MODE_HOME_LIMIT THEN
                    G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;
                ELSIF eTargetMode = E_OperatingMode#MODE_HOME_EOT THEN
                    G_sysCurrentState := E_SystemState#ST_HOME_EOT;
                ELSIF eTargetMode = E_OperatingMode#MODE_IDLE THEN
                    G_sysCurrentState := E_SystemState#ST_BRAKE_ENGAGE;
                END_IF;
            END_IF;

        ELSIF tHandshakeTimer.Q THEN
            G_sysActiveFault := E_FaultCode#FAULT_HANDSHAKE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - DEACTIVATION
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE THEN

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            tBrakeTimer(IN := FALSE);
        END_IF;

        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;
            G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_DISABLE THEN

        IF NOT G_staPower.Status THEN
            G_sysDriveEnabled := FALSE;
            G_sysDriveReady := FALSE;
            G_sysCurrentState := E_SystemState#ST_IDLE;
        END_IF;


    (* ======================================================================
       FAULT STATES
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_FAULT THEN
        (* ----------------------------------------------------------------
           FAULT STATE - Active fault handling

           NO brake engagement here - drive holds position via MC_Stop.
           Two exit paths:
           - Fast recovery: Handshake before timeout -> ST_BRAKE_HOLD (drive stays on)
           - Slow recovery: Timeout -> ST_FAULT_IDLE (drive off, brake on)
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_doInMotion := FALSE;

            G_sysLastFaultCode := G_sysActiveFault;
            G_sysFaultCount := G_sysFaultCount + UDINT#1;

            tStateTimer(IN := FALSE);
        END_IF;

        tStateTimer(IN := TRUE, PT := G_cfgFaultIdleTimeout);

        IF fbFaultReset.FaultResetValid THEN
            G_sysActiveFault := E_FaultCode#FAULT_NONE;
            G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;

        ELSIF tStateTimer.Q THEN
            G_sysCurrentState := E_SystemState#ST_FAULT_IDLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
        (* ----------------------------------------------------------------
           FAULT IDLE STATE - Drive disabled, brake engaged

           Entered after ST_FAULT timeout (slow recovery path).
           Awaits fault reset handshake to return to normal ST_IDLE.

           Any fault invalidates trust in position, so both homing flags
           are re-asserted on exit - the operator must re-home before
           resuming operational modes.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            tBrakeTimer(IN := FALSE);
            G_doInMotion := FALSE;
        END_IF;

        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;
            IF NOT G_staPower.Status THEN
                G_sysDriveEnabled := FALSE;
            END_IF;
        END_IF;

        IF fbFaultReset.FaultResetValid THEN
            G_sysActiveFault := E_FaultCode#FAULT_NONE;
            G_flagAbsHomeRequired := TRUE;
            G_flagEOTHomeRequired := TRUE;
            G_sysCurrentState := E_SystemState#ST_IDLE;
        END_IF;

END_IF;


(* --------------------------------------------------------------------------
   Command Arbitration (ST -> LD POU via G_cmd*)
   Default all commands to idle, then override per state.
   -------------------------------------------------------------------------- *)

(* Zero all commands *)
G_cmdPower.Enable := FALSE;
G_cmdPower.EnablePositive := FALSE;
G_cmdPower.EnableNegative := FALSE;
G_cmdStop.Execute := FALSE;
G_cmdStop.Deceleration := LREAL#0.0;
G_cmdReset.Execute := FALSE;
G_cmdMoveAbsolute.Execute := FALSE;
G_cmdMoveAbsolute.Position := LREAL#0.0;
G_cmdMoveAbsolute.Velocity := LREAL#0.0;
G_cmdMoveAbsolute.Acceleration := LREAL#0.0;
G_cmdMoveAbsolute.Deceleration := LREAL#0.0;
G_cmdMoveAbsolute.Direction := 0;
G_cmdMoveVelocity.Execute := FALSE;
G_cmdMoveVelocity.Velocity := LREAL#0.0;
G_cmdMoveVelocity.Acceleration := LREAL#0.0;
G_cmdMoveVelocity.Deceleration := LREAL#0.0;
G_cmdMoveVelocity.Direction := 0;
G_cmdDirectControl.Enable := FALSE;
G_cmdDirectControl.ControlMode := 0;
G_cmdDirectControl.Position := LREAL#0.0;
G_cmdDirectControl.Velocity := LREAL#0.0;
G_cmdDirectControl.Torque := LREAL#0.0;
G_cmdSetPosition.Execute := FALSE;
G_cmdSetPosition.Position := LREAL#0.0;
G_cmdSetPosition.Mode := FALSE;

(* MC_Power: enable for all drive-active states *)
IF (G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE) OR
   (G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE) OR
   (G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD) OR
   (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
   (G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
   (G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL) OR
   (G_sysCurrentState = E_SystemState#ST_GO_HOME) OR
   (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT) OR
   (G_sysCurrentState = E_SystemState#ST_HOME_EOT) OR
   (G_sysCurrentState = E_SystemState#ST_HOME_COMPLETE) OR
   (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) OR
   (G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE) OR
   (G_sysCurrentState = E_SystemState#ST_FAULT) THEN
    G_cmdPower.Enable := TRUE;
    G_cmdPower.EnablePositive := TRUE;
    G_cmdPower.EnableNegative := TRUE;
ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
    IF NOT tBrakeTimer.Q THEN
        G_cmdPower.Enable := TRUE;
        G_cmdPower.EnablePositive := TRUE;
        G_cmdPower.EnableNegative := TRUE;
    END_IF;
END_IF;

(* State-direct commands for built-in FBs *)
IF G_sysCurrentState = E_SystemState#ST_POSITION_CTRL THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := 1;
    G_cmdDirectControl.Position := G_sysCommandedPosition;
    G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;

ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := 2;
    G_cmdDirectControl.Velocity := G_sysCommandedVelocity;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;

ELSIF G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := 3;
    G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
    G_cmdDirectControl.Torque := G_sysCommandedTorque;

ELSIF G_sysCurrentState = E_SystemState#ST_HOLD_POSITION THEN
    G_cmdStop.Execute := NOT G_sysHalted;

ELSIF G_sysCurrentState = E_SystemState#ST_FAULT THEN
    G_cmdStop.Execute := NOT G_staStop.Done;

END_IF;


(* --------------------------------------------------------------------------
   Function Block Calls - Exactly one call per instance
   -------------------------------------------------------------------------- *)

(* Input filters *)
fbFilterModeBit0(RawInput := G_diModeBit0, DebounceTime := G_cfgDebounceTimeMode);
fbFilterModeBit1(RawInput := G_diModeBit1, DebounceTime := G_cfgDebounceTimeMode);
fbFilterModeBit2(RawInput := G_diModeBit2, DebounceTime := G_cfgDebounceTimeMode);
fbFilterMotionEnable(RawInput := G_diMotionEnable, DebounceTime := G_cfgDebounceTimeMotion);
fbFilterLimitRetract(RawInput := G_diLimitRetract, DebounceTime := G_cfgDebounceTimeLimitStd);
fbFilterLimitHome(RawInput := G_diLimitHome, DebounceTime := G_cfgDebounceTimeLimitHome);
fbFilterFaultReset(RawInput := G_diFaultReset, DebounceTime := G_cfgDebounceTimeFault);

fbFilterAnalog(
    Enable := TRUE,
    RawInput := G_aiReference,
    Alpha := rAnalogFilterAlpha,
    MedianFilterSize := G_cfgAnalogMedianSize
);

(* Mode decoder *)
fbModeDecoder(
    Enable := TRUE,
    ModeBit0 := bFilteredModeBit0,
    ModeBit1 := bFilteredModeBit1,
    ModeBit2 := bFilteredModeBit2
);

(* Analog processor *)
fbAnalogProc(
    Enable := TRUE,
    RawAnalogInput := nFilteredAnalog,
    CurrentMode := G_sysRequestedMode,
    Stage1PosMin := G_cfgPosMapStage1PosMin,
    Stage1PosMax := G_cfgPosMapTransitionPos,
    Stage1VoltMin := G_cfgPosMapStage1VoltMin,
    Stage1VoltMax := G_cfgPosMapTransitionVolt,
    Stage2PosMin := G_cfgPosMapTransitionPos,
    Stage2PosMax := G_cfgPosMapStage2PosMax,
    Stage2VoltMin := G_cfgPosMapTransitionVolt,
    Stage2VoltMax := G_cfgPosMapStage2VoltMax,
    VelMin := G_cfgAnalogVelMin,
    VelMax := G_cfgAnalogVelMax,
    TorqueMin := G_cfgAnalogTorqueMin,
    TorqueMax := G_cfgAnalogTorqueMax
);

(* Handshake manager *)
fbHandshake(
    Enable := (G_sysCurrentState = E_SystemState#ST_IDLE) OR
              (G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD) OR
              (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION),
    RequestedMode := G_sysRequestedMode,
    ModeStable := fbModeDecoder.ModeStable,
    MotionEnable := bFilteredMotionEnable,
    HandshakeTimeout := G_cfgHandshakeTimeout
);

(* Fault reset handler - parameters differ by state *)
IF G_sysCurrentState = E_SystemState#ST_FAULT THEN
    bFaultResetInFaultState := TRUE;
    eFaultResetActiveCode := G_sysActiveFault;
ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
    bFaultResetInFaultState := (G_sysActiveFault <> E_FaultCode#FAULT_NONE);
    eFaultResetActiveCode := G_sysLastFaultCode;
ELSE
    bFaultResetInFaultState := FALSE;
    eFaultResetActiveCode := E_FaultCode#FAULT_NONE;
END_IF;

fbFaultReset(
    Enable := (G_sysCurrentState = E_SystemState#ST_FAULT) OR
              (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE),
    FaultReset := bFilteredFaultReset,
    MotionEnable := bFilteredMotionEnable,
    InFaultState := bFaultResetInFaultState,
    BitsStable := fbModeDecoder.ModeStable,
    ActiveFaultCode := eFaultResetActiveCode,
    MasterFaultCode := FN_BitsToFaultCode(bFilteredModeBit0, bFilteredModeBit1, bFilteredModeBit2)
);

(* Go Home handler *)
fbGoHome(
    Enable := (G_sysCurrentState = E_SystemState#ST_GO_HOME),
    Execute := (G_sysCurrentState = E_SystemState#ST_GO_HOME),
    AbsHomeRequired := G_flagAbsHomeRequired,
    EOTHomeRequired := G_flagEOTHomeRequired,
    HomePosition := G_cfgGoHomePosition - G_posEOTOffset,
    HomeVelocity := G_cfgGoHomeVelocity,
    Abort := fTrigMotionEnable.Q,
    StaMoveAbsolute := G_staMoveAbsolute,
    StaStop := G_staStop
);

(* Home to Limit Switch handler *)
fbHomeLimit(
    Enable := (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT),
    Execute := (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT),
    ApproachVelocity := G_cfgHomeLimApproachVel,
    BackoffVelocity := G_cfgVelHomingSlow,
    SetPosition := G_cfgHomeLimSetPosition,
    Timeout := G_cfgHomingTimeout,
    LimitHomeActive := G_sysLimitHomeActive,
    LimitRetractActive := G_sysLimitRetractActive,
    Abort := fTrigMotionEnable.Q,
    StaMoveVelocity := G_staMoveVelocity,
    StaStop := G_staStop,
    StaSetPosition := G_staSetPosition
);

(* Home to End-of-Travel handler *)
fbHomeEOT(
    Enable := (G_sysCurrentState = E_SystemState#ST_HOME_EOT),
    Execute := (G_sysCurrentState = E_SystemState#ST_HOME_EOT),
    AbsHomeRequired := G_flagAbsHomeRequired,
    FastVelocity := G_cfgHomeEOTFastVel,
    SlowVelocity := G_cfgHomeEOTSlowVel,
    TorqueLimit := G_cfgTorqueHomingLimit,
    TorqueThreshold := G_cfgHomeEOTTorqueThresh,
    ApproachDistance := G_cfgHomeEOTApproachDist,
    ExpectedEOTPosition := G_cfgHomeEOTSetPosition,
    StallDetectTime := G_cfgStallDetectTime,
    Timeout := G_cfgHomingTimeout,
    ActualPosition := G_sysActualPosition,
    ActualVelocity := G_sysActualVelocity,
    ActualTorque := G_sysActualTorque,
    Abort := fTrigMotionEnable.Q,
    StaDirectControl := G_staDirectControl,
    StaStop := G_staStop
);

(* Safety monitors *)
fbPistonExitGuard(
    Enable := TRUE,
    ActualPosition := G_sysActualPosition,
    ActualVelocity := G_sysActualVelocity,
    ActualTorque := G_sysActualTorque,
    SoftLimitMin := G_cfgPosSoftLimitMin,
    GuardMargin := G_cfgPosExitGuardMargin,
    VelocityThreshold := G_cfgInMotionVelThreshold,
    TorqueThreshold := G_cfgTorqueHomingThreshold
);

fbSafetyMonitor(
    Enable := TRUE,
    LimitRetractActive := G_sysLimitRetractActive,
    LimitHomeActive := G_sysLimitHomeActive,
    ActualPosition := G_sysActualPosition,
    SoftLimitMin := G_cfgPosSoftLimitMin,
    SoftLimitMax := G_cfgPosSoftLimitMax,
    DriveReady := G_sysDriveReady,
    DriveFault := G_sysDriveFault,
    CurrentState := G_sysCurrentState,
    CurrentMode := G_sysConfirmedMode,
    ExitGuardTriggered := fbPistonExitGuard.FaultTriggered
);

(* Output multiplexer *)
fbOutputMux(
    Enable := TRUE,
    FaultActive := (G_sysCurrentState = E_SystemState#ST_FAULT) OR
                   (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE),
    FaultCode := G_sysActiveFault,
    ConfirmedMode := fbHandshake.ConfirmedMode
);

(* Position output *)
fbPosOutput(
    Enable := TRUE,
    ActualPosition := G_sysActualPosition + G_posEOTOffset,
    Stage1PosMin := G_cfgPosMapStage1PosMin,
    Stage1PosMax := G_cfgPosMapTransitionPos,
    Stage1VoltMin := G_cfgPosMapStage1VoltMin,
    Stage1VoltMax := G_cfgPosMapTransitionVolt,
    Stage2PosMin := G_cfgPosMapTransitionPos,
    Stage2PosMax := G_cfgPosMapStage2PosMax,
    Stage2VoltMin := G_cfgPosMapTransitionVolt,
    Stage2VoltMax := G_cfgPosMapStage2VoltMax
);


(* --------------------------------------------------------------------------
   Route Active FB Command Outputs -> G_cmd*
   Overrides the zeroed defaults from Command Arbitration for states
   where a user FB owns the motion command.
   -------------------------------------------------------------------------- *)

IF G_sysCurrentState = E_SystemState#ST_GO_HOME THEN
    G_cmdMoveAbsolute := fbGoHome.CmdMoveAbsolute;
    G_cmdStop := fbGoHome.CmdStop;

ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIMIT THEN
    G_cmdMoveVelocity := fbHomeLimit.CmdMoveVelocity;
    G_cmdStop := fbHomeLimit.CmdStop;
    G_cmdSetPosition := fbHomeLimit.CmdSetPosition;

ELSIF G_sysCurrentState = E_SystemState#ST_HOME_EOT THEN
    G_cmdDirectControl := fbHomeEOT.CmdDirectControl;
    G_cmdStop := fbHomeEOT.CmdStop;

END_IF;


(* --------------------------------------------------------------------------
   Safety Monitoring
   Runs after FB calls so safety FBs have fresh outputs.
   -------------------------------------------------------------------------- *)

G_sysExitGuardActive := fbPistonExitGuard.GuardActive;
G_sysInCylinderZone := NOT fbPistonExitGuard.InDangerZone;

IF fbSafetyMonitor.FaultDetected AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT) AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT_IDLE) THEN
    G_sysActiveFault := fbSafetyMonitor.FaultCode;
    G_sysCurrentState := E_SystemState#ST_FAULT;
END_IF;


(* --------------------------------------------------------------------------
   Update Outputs
   -------------------------------------------------------------------------- *)

G_doModeConfBit0 := fbOutputMux.DO0_Output;
G_doModeConfBit1 := fbOutputMux.DO1_Output;
G_doModeConfBit2 := fbOutputMux.DO2_Output;
G_doFaultActive := fbOutputMux.FaultActiveOutput;

G_aoPositionOutput := fbPosOutput.RawAnalogOutput;

G_sysCycleCount := G_sysCycleCount + UDINT#1;
