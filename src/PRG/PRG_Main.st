(* ============================================================================
   PRG_Main - Main Program with State Machine

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Main program containing the central state machine for servo control.
   Coordinates all function blocks, handles state transitions, and manages
   the handshake protocol with the master controller.

   State Machine Groups:
   - Initialization: ST_INIT, ST_ENCODER_CHECK, ST_REQUIRE_ABS_HOME
   - Idle: ST_IDLE
   - Transition (Activation): ST_DRIVE_ENABLE, ST_BRAKE_RELEASE
   - Operational: ST_BRAKE_HOLD, ST_POSITION_CTRL, ST_VELOCITY_CTRL,
                  ST_TORQUE_CTRL, ST_GO_HOME, ST_HOME_*
   - Transition (Inter-Mode): ST_HOLD_POSITION
   - Transition (Deactivation): ST_BRAKE_ENGAGE, ST_DRIVE_DISABLE
   - Fault: ST_FAULT, ST_FAULT_RECOVERY

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
       fbEncoderManager    : FB_EncoderManager
       fbPower             : MC_Power
       fbReadActualPos     : MC_ReadActualPosition
       fbReadActualVel     : MC_ReadActualVelocity
       fbReadActualTorque  : MC_ReadActualTorque
       fbStop              : MC_Stop
       fbReset             : MC_Reset
       fbSetPosition       : MC_SetPosition
       fbMoveAbsolute      : MC_MoveAbsolute
       fbMoveVelocity      : MC_MoveVelocity
       fbDirectControl     : Y_DirectControl
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
       nFilteredAnalog     : INT
       rAnalogFilterAlpha  : LREAL := 0.0
   ============================================================================ *)

(* ============================================================================
   MAIN PROGRAM BODY
   ============================================================================ *)

(* --------------------------------------------------------------------------
   First Scan Initialization
   -------------------------------------------------------------------------- *)
IF bFirstScan THEN
    bFirstScan := FALSE;

    (* Initialize state machine *)
    G_sysCurrentState := E_SystemState#ST_INIT;
    G_sysPreviousState := E_SystemState#ST_INIT;

    (* Initialize homing flags *)
    G_flagEOTHomeRequired := TRUE;    (* Always required on power-up *)
    G_flagAbsHomeRequired := FALSE;   (* Will be set by encoder check *)

    (* Initialize outputs to safe state *)
    G_doBrakeDisengage := FALSE;      (* Brake engaged *)
    G_doPerformanceStatus := FALSE;
    G_doHomingComplete := FALSE;
    G_doInMotion := FALSE;

    (* Calculate analog filter alpha: alpha = dt / (tau + dt)
       dt = 1ms (fixed task cycle time from MP2600iec hardware config)
       tau = G_cfgAnalogFilterTimeConst *)
    IF G_cfgAnalogFilterTimeConst > T#0MS THEN
        rAnalogFilterAlpha := 0.001 / ((DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / 1000.0) + 0.001);
    ELSE
        rAnalogFilterAlpha := 1.0;  (* No filtering if time constant is zero *)
    END_IF;
    rAnalogFilterAlpha := LIMIT(0.0, rAnalogFilterAlpha, 1.0);
END_IF;


(* --------------------------------------------------------------------------
   Filter Digital Inputs (Debouncing)
   -------------------------------------------------------------------------- *)

(* Mode bits - standard debounce *)
fbFilterModeBit0(RawInput := G_diModeBit0, DebounceTime := G_cfgDebounceTimeMode);
fbFilterModeBit1(RawInput := G_diModeBit1, DebounceTime := G_cfgDebounceTimeMode);
fbFilterModeBit2(RawInput := G_diModeBit2, DebounceTime := G_cfgDebounceTimeMode);
bFilteredModeBit0 := fbFilterModeBit0.FilteredOutput;
bFilteredModeBit1 := fbFilterModeBit1.FilteredOutput;
bFilteredModeBit2 := fbFilterModeBit2.FilteredOutput;

(* Motion enable - shorter debounce for responsiveness *)
fbFilterMotionEnable(RawInput := G_diMotionEnable, DebounceTime := G_cfgDebounceTimeMotion);
bFilteredMotionEnable := fbFilterMotionEnable.FilteredOutput;

(* Limit switches - short debounce for responsiveness *)
fbFilterLimitRetract(RawInput := G_diLimitRetract, DebounceTime := G_cfgDebounceTimeLimitStd);
fbFilterLimitHome(RawInput := G_diLimitHome, DebounceTime := G_cfgDebounceTimeLimitHome);
bFilteredLimitRetract := fbFilterLimitRetract.FilteredOutput;
bFilteredLimitHome := fbFilterLimitHome.FilteredOutput;

(* Fault reset *)
fbFilterFaultReset(RawInput := G_diFaultReset, DebounceTime := G_cfgDebounceTimeFault);
bFilteredFaultReset := fbFilterFaultReset.FilteredOutput;


(* --------------------------------------------------------------------------
   Filter Analog Input (Median + Low-Pass)
   -------------------------------------------------------------------------- *)

fbFilterAnalog(
    Enable := TRUE,
    RawInput := G_aiReference,
    Alpha := rAnalogFilterAlpha,
    MedianFilterSize := G_cfgAnalogMedianSize
);
nFilteredAnalog := fbFilterAnalog.FilteredOutput;


(* --------------------------------------------------------------------------
   Read Inputs and Update Feedback
   -------------------------------------------------------------------------- *)

(* Decode mode from filtered digital inputs *)
fbModeDecoder(
    Enable := TRUE,
    ModeBit0 := bFilteredModeBit0,
    ModeBit1 := bFilteredModeBit1,
    ModeBit2 := bFilteredModeBit2
);
G_sysRequestedMode := fbModeDecoder.DecodedMode;

(* Edge detection on filtered motion enable *)
rTrigMotionEnable(CLK := bFilteredMotionEnable);
fTrigMotionEnable(CLK := bFilteredMotionEnable);

(* Read actual position, velocity, torque from drive *)
fbReadActualPos(Axis := G_sysAxis, Enable := TRUE);
fbReadActualVel(Axis := G_sysAxis, Enable := TRUE);
fbReadActualTorque(Axis := G_sysAxis, Enable := TRUE);

IF fbReadActualPos.Valid THEN
    G_sysActualPosition := fbReadActualPos.Position;
END_IF;
IF fbReadActualVel.Valid THEN
    G_sysActualVelocity := fbReadActualVel.ActualVelocity;
END_IF;
IF fbReadActualTorque.Valid THEN
    G_sysActualTorque := fbReadActualTorque.ActualTorque;
END_IF;

(* Update limit switch status from filtered inputs (PNP NC: FALSE = triggered) *)
G_sysLimitRetractActive := NOT bFilteredLimitRetract;
G_sysLimitHomeActive := NOT bFilteredLimitHome;

(* Process filtered analog input - uses two-stage mapping for position reference *)
fbAnalogProc(
    Enable := TRUE,
    RawAnalogInput := nFilteredAnalog,
    CurrentMode := G_sysRequestedMode,
    (* Two-stage position mapping (same config as position output) *)
    Stage1PosMin := G_cfgPosMapStage1PosMin,
    Stage1PosMax := G_cfgPosMapTransitionPos,
    Stage1VoltMin := G_cfgPosMapStage1VoltMin,
    Stage1VoltMax := G_cfgPosMapTransitionVolt,
    Stage2PosMin := G_cfgPosMapTransitionPos,
    Stage2PosMax := G_cfgPosMapStage2PosMax,
    Stage2VoltMin := G_cfgPosMapTransitionVolt,
    Stage2VoltMax := G_cfgPosMapStage2VoltMax,
    (* Linear velocity and torque scaling *)
    VelMin := G_cfgAnalogVelMin,
    VelMax := G_cfgAnalogVelMax,
    TorqueMin := G_cfgAnalogTorqueMin,
    TorqueMax := G_cfgAnalogTorqueMax
);


(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

(* Detect state entry *)
bEnterState := (G_sysCurrentState <> G_sysPreviousState);
IF bEnterState THEN
    (* Capture fault origin before G_sysPreviousState is overwritten *)
    IF G_sysCurrentState = E_SystemState#ST_FAULT THEN
        G_sysLastFaultState := G_sysPreviousState;
    END_IF;
    G_sysPreviousState := G_sysCurrentState;
    tStateTimer(IN := FALSE);   (* Reset state timer on entry *)
END_IF;

IF G_sysCurrentState = E_SystemState#ST_INIT THEN

    (* ======================================================================
       INITIALIZATION STATES
       ====================================================================== *)

    (* E_SystemState#ST_INIT *)
        (* Power-on initialization *)
        IF bEnterState THEN
            (* Ensure safe output state *)
            G_doBrakeDisengage := FALSE;
            fbPower(Axis := G_sysAxis, Enable := FALSE);
        END_IF;

        (* Proceed to encoder check *)
        G_sysCurrentState := E_SystemState#ST_ENCODER_CHECK;


ELSIF G_sysCurrentState = E_SystemState#ST_ENCODER_CHECK THEN
        (* Validate absolute encoder status using AbsolutePositionManager *)
        IF bEnterState THEN
            fbEncoderManager(Enable := FALSE);  (* Reset *)
        END_IF;

        (* Run encoder check *)
        fbEncoderManager(
            Enable := TRUE,
            Axis := G_sysAxis,
            CheckOnStartup := TRUE
        );

        IF fbEncoderManager.CheckComplete THEN
            (* Check complete - read results *)
            G_sysEncoderValid := fbEncoderManager.EncoderValid;
            G_sysEncoderBatteryOK := fbEncoderManager.BatteryOK;

            IF G_sysEncoderValid THEN
                (* Encoder valid - proceed to idle *)
                G_flagAbsHomeRequired := FALSE;
                G_sysCurrentState := E_SystemState#ST_IDLE;
            ELSE
                (* Encoder invalid - set flag and proceed *)
                G_sysCurrentState := E_SystemState#ST_REQUIRE_ABS_HOME;
            END_IF;

        ELSIF fbEncoderManager.Error THEN
            (* Encoder check error - assume invalid, require homing *)
            G_sysEncoderValid := FALSE;
            G_sysCurrentState := E_SystemState#ST_REQUIRE_ABS_HOME;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_REQUIRE_ABS_HOME THEN
        (* Set absolute home requirement flag *)
        IF bEnterState THEN
            G_flagAbsHomeRequired := TRUE;
        END_IF;

        (* Proceed to idle *)
        G_sysCurrentState := E_SystemState#ST_IDLE;


    (* ======================================================================
       IDLE STATE
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_IDLE THEN
        IF bEnterState THEN
            (* Ensure safe state *)
            G_doBrakeDisengage := FALSE;  (* Brake engaged *)
            G_sysDriveEnabled := FALSE;
            G_sysDriveReady := FALSE;
            fbPower(Axis := G_sysAxis, Enable := FALSE);
            (* NOTE: G_sysConfirmedMode is used by fbHandshake to set DO0-2 output bits.
               It represents the mode the slave is confirming to the master. *)
            G_sysConfirmedMode := E_OperatingMode#MODE_IDLE;
        END_IF;

        (* Update handshake manager - it handles mode confirmation output *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := G_sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            HandshakeTimeout := G_cfgHandshakeTimeout
        );

        (* Wait for handshake to complete before transitioning *)
        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            (* Check if this is an operational mode requiring homing *)
            IF FN_IsOperationalMode(eTargetMode) THEN
                (* Check homing requirements *)
                IF G_flagAbsHomeRequired OR G_flagEOTHomeRequired THEN
                    (* Homing required but not satisfied - enter fault *)
                    G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                    G_sysCurrentState := E_SystemState#ST_FAULT;
                ELSE
                    (* Requirements met - proceed to drive enable *)
                    G_sysCurrentState := E_SystemState#ST_DRIVE_ENABLE;
                END_IF;

            ELSIF FN_IsHomingMode(eTargetMode) THEN
                (* Homing modes - check ordering requirements *)
                IF (eTargetMode = E_OperatingMode#MODE_HOME_EOT) AND G_flagAbsHomeRequired THEN
                    (* EOT homing requires limit switch homing first if encoder invalid *)
                    G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                    G_sysCurrentState := E_SystemState#ST_FAULT;
                ELSE
                    (* Homing mode allowed - proceed *)
                    G_sysCurrentState := E_SystemState#ST_DRIVE_ENABLE;
                END_IF;

            ELSE
                (* Unknown mode - stay in idle *)
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

        (* Enable drive power and wait for ready *)
        fbPower(Axis := G_sysAxis, Enable := TRUE,
                Enable_Positive := TRUE, Enable_Negative := TRUE);

        (* Wait for drive ready with timeout *)
        tDriveReadyTimer(IN := TRUE, PT := G_cfgDriveReadyTimeout);

        IF fbPower.Status THEN
            (* Drive is ready *)
            G_sysDriveEnabled := TRUE;
            G_sysDriveReady := TRUE;
            G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;

        ELSIF fbPower.Error OR tDriveReadyTimer.Q THEN
            (* Drive error or timeout *)
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE THEN
        IF bEnterState THEN
            G_doBrakeDisengage := TRUE;   (* Release brake *)
            tBrakeTimer(IN := FALSE);
        END_IF;

        (* Wait for brake release delay *)
        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeDisengageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := FALSE;

            (* Transition to target operational state *)
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
                (* Unknown mode - brake is released, hold position until handshake *)
                G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
            END_IF;
        END_IF;


    (* ======================================================================
       OPERATIONAL STATES
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD THEN
        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;  (* Engage brake *)
            G_sysConfirmedMode := E_OperatingMode#MODE_BRAKE_HOLD;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        G_doInMotion := FALSE;

        (* Update handshake manager for mode changes *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := G_sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            HandshakeTimeout := G_cfgHandshakeTimeout
        );

        (* Allow mode changes when handshake completes *)
        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            (* Check homing requirements for operational modes *)
            IF FN_IsOperationalMode(eTargetMode) AND
               (G_flagAbsHomeRequired OR G_flagEOTHomeRequired) THEN
                G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                G_sysCurrentState := E_SystemState#ST_FAULT;
            ELSE
                IF eTargetMode = E_OperatingMode#MODE_IDLE THEN
                    (* Return to idle - disable drive *)
                    G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;

                ELSIF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                    (* Stay in brake hold - do nothing *)
                    ;

                ELSIF eTargetMode = E_OperatingMode#MODE_POSITION OR eTargetMode = E_OperatingMode#MODE_VELOCITY OR eTargetMode = E_OperatingMode#MODE_TORQUE OR
                      eTargetMode = E_OperatingMode#MODE_GO_HOME OR eTargetMode = E_OperatingMode#MODE_HOME_LIMIT OR eTargetMode = E_OperatingMode#MODE_HOME_EOT THEN
                    (* Other operational modes - need to release brake first *)
                    G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;
                END_IF;
            END_IF;
        END_IF;

        (* Note: E_SystemState#ST_BRAKE_HOLD does NOT transition to E_SystemState#ST_HOLD_POSITION on motion enable drop.
           It is safer to keep the brake engaged. Mode changes are handled via handshake above. *)


ELSIF G_sysCurrentState = E_SystemState#ST_POSITION_CTRL THEN
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_POSITION;
            (* Configure position mode limits *)
            stCurrentLimits.PosMax := G_cfgPosSoftLimitMax;
            stCurrentLimits.PosMin := G_cfgPosSoftLimitMin;
            stCurrentLimits.VelMax := G_cfgVelLimitNormal;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        (* Get position reference from analog input and apply EOT offset *)
        (* Master commands in master coordinates, convert to slave coordinates *)
        (* Slave = Master - Offset *)
        G_sysCommandedPosition := fbAnalogProc.ScaledPosition - G_posEOTOffset;

        (* Apply position limits - additional safety layer *)
        G_sysCommandedPosition := LIMIT(stCurrentLimits.PosMin,
                                      G_sysCommandedPosition,
                                      stCurrentLimits.PosMax);

        (* Execute position control via Y_DirectControl *)
        fbDirectControl(Axis := G_sysAxis,
                        Enable := TRUE,
                        ControlMode := 1,  (* Position mode *)
                        Position := G_sysCommandedPosition,
                        Velocity := stCurrentLimits.VelMax,
                        Torque := stCurrentLimits.TorqueMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        (* Check for motion enable drop *)
        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_VELOCITY;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        (* Get velocity reference from analog input *)
        G_sysCommandedVelocity := fbAnalogProc.ScaledVelocity;

        (* Apply velocity limits *)
        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      G_sysCommandedVelocity,
                                      stCurrentLimits.VelMax);

        (* Execute velocity control *)
        fbDirectControl(Axis := G_sysAxis,
                        Enable := TRUE,
                        ControlMode := 2,  (* Velocity mode *)
                        Velocity := G_sysCommandedVelocity,
                        Torque := stCurrentLimits.TorqueMax);

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

        (* Get torque reference from analog input *)
        G_sysCommandedTorque := fbAnalogProc.ScaledTorque;

        (* Apply torque limits *)
        G_sysCommandedTorque := LIMIT(-stCurrentLimits.TorqueMax,
                                    G_sysCommandedTorque,
                                    stCurrentLimits.TorqueMax);

        (* Execute torque control *)
        fbDirectControl(Axis := G_sysAxis,
                        Enable := TRUE,
                        ControlMode := 3,  (* Torque mode *)
                        Velocity := stCurrentLimits.VelMax,  (* As limit *)
                        Torque := G_sysCommandedTorque);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_GO_HOME THEN
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_GO_HOME;
            fbGoHome(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Use FB_GoHome for homing requirement checking and move *)
        (* Apply offset: Slave position = Master position - Offset *)
        fbGoHome(
            Enable := TRUE,
            Execute := TRUE,
            Axis := G_sysAxis,
            AbsHomeRequired := G_flagAbsHomeRequired,
            EOTHomeRequired := G_flagEOTHomeRequired,
            HomePosition := G_cfgGoHomePosition - G_posEOTOffset,
            HomeVelocity := G_cfgGoHomeVelocity,
            Abort := fTrigMotionEnable.Q
        );

        G_doInMotion := fbGoHome.InMotion;

        IF fbGoHome.RedirectToHoming THEN
            (* Need limit switch homing first - redirect *)
            fbGoHome(Enable := FALSE);
            G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;

        ELSIF fbGoHome.Done THEN
            (* At home position - stay here until mode change *)
            G_doInMotion := FALSE;

        ELSIF fbGoHome.Error THEN
            (* Error during move *)
            fbGoHome(Enable := FALSE);
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            (* Note: ELSIF ensures RedirectToHoming is FALSE at this point *)
            fbGoHome(Enable := FALSE);
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIMIT THEN
        (* Mode 110 - Home to Limit Switch
           Uses FB_HomeLimit to encapsulate the complete homing sequence:
           approach, over-travel handling, detect, backoff, and set reference *)
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_HOME_LIMIT;
            G_flagHomingInProgress := TRUE;
            fbHomeLimit(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Call homing FB *)
        fbHomeLimit(
            Enable := TRUE,
            Execute := TRUE,
            Axis := G_sysAxis,
            ApproachVelocity := G_cfgHomeLimApproachVel,
            BackoffVelocity := G_cfgVelHomingSlow,
            SetPosition := G_cfgHomeLimSetPosition,
            Timeout := G_cfgHomingTimeout,
            LimitHomeActive := G_sysLimitHomeActive,
            LimitRetractActive := G_sysLimitRetractActive,
            Abort := fTrigMotionEnable.Q
        );

        G_doInMotion := fbHomeLimit.InMotion;

        IF fbHomeLimit.Done THEN
            (* Homing complete - clear requirement and restore encoder validity *)
            fbHomeLimit(Enable := FALSE);
            G_flagAbsHomeRequired := FALSE;
            G_sysEncoderValid := TRUE;
            G_flagHomingComplete := TRUE;
            G_sysCurrentState := E_SystemState#ST_HOME_COMPLETE;

        ELSIF fbHomeLimit.Error THEN
            (* Homing failed *)
            fbHomeLimit(Enable := FALSE);
            G_sysActiveFault := E_FaultCode#FAULT_LIMIT_SWITCH;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            fbHomeLimit(Enable := FALSE);
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIM_APPROACH OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_DETECT OR
      G_sysCurrentState = E_SystemState#ST_HOME_LIM_BACKOFF OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_SETREF THEN
        (* These sub-states are now handled internally by FB_HomeLimit.
           If we get here, redirect back to E_SystemState#ST_HOME_LIMIT *)
        G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_EOT THEN
        (* Mode 111 - Home to End-of-Travel
           Uses FB_HomeEOT to encapsulate the complete homing sequence:
           check requirements, fast approach, slow approach, stall detect, offset calc *)
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_HOME_EOT;
            G_flagHomingInProgress := TRUE;
            fbHomeEOT(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Call homing FB *)
        fbHomeEOT(
            Enable := TRUE,
            Execute := TRUE,
            Axis := G_sysAxis,
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
            Abort := fTrigMotionEnable.Q
        );

        G_doInMotion := fbHomeEOT.InMotion;

        IF fbHomeEOT.RequiresLimitHome THEN
            (* Must do limit switch homing before EOT homing *)
            fbHomeEOT(Enable := FALSE);
            G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fbHomeEOT.Done THEN
            (* EOT homing complete - get results and clear requirement *)
            G_posActualEOT := fbHomeEOT.EOTPosition;
            G_posEOTOffset := fbHomeEOT.EOTOffset;
            fbHomeEOT(Enable := FALSE);
            G_flagEOTHomeRequired := FALSE;
            G_flagHomingComplete := TRUE;
            G_sysCurrentState := E_SystemState#ST_HOME_COMPLETE;

        ELSIF fbHomeEOT.Error THEN
            (* Homing failed *)
            fbHomeEOT(Enable := FALSE);
            G_sysActiveFault := E_FaultCode#FAULT_POSITION;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            fbHomeEOT(Enable := FALSE);
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
ELSIF G_sysCurrentState = E_SystemState#ST_HOME_EOT_FAST OR G_sysCurrentState = E_SystemState#ST_HOME_EOT_SLOW OR
      G_sysCurrentState = E_SystemState#ST_HOME_EOT_DETECT OR G_sysCurrentState = E_SystemState#ST_HOME_EOT_SETREF THEN
        (* These sub-states are now handled internally by FB_HomeEOT.
           If we get here, redirect back to E_SystemState#ST_HOME_EOT *)
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
            (* IMPORTANT: Order matters here!
               1. fbStop FIRST - initiates controlled deceleration and transitions axis
                  to PLCopen "Standstill" state which maintains active position control
               2. fbDirectControl SECOND - releases Y_DirectControl (Yaskawa-specific)

               If reversed, Y_DirectControl would release servo reference while still
               moving, potentially causing a drive fault. MC_Stop must take over
               position control BEFORE Y_DirectControl releases it. *)
            fbStop(Axis := G_sysAxis, Execute := TRUE);
            fbDirectControl(Axis := G_sysAxis, Enable := FALSE);
            G_sysHalted := FALSE;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        (* Wait for halt to complete *)
        IF NOT G_sysHalted THEN
            fbStop(Axis := G_sysAxis, Execute := TRUE);
            IF fbStop.Done THEN
                G_sysHalted := TRUE;
                fbStop(Execute := FALSE);
            END_IF;
        END_IF;

        G_doInMotion := NOT G_sysHalted;

        (* Update handshake manager *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := G_sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            HandshakeTimeout := G_cfgHandshakeTimeout
        );

        (* Wait for new mode handshake *)
        tHandshakeTimer(IN := TRUE, PT := G_cfgHandshakeTimeout);

        IF fbHandshake.HandshakeComplete THEN
            (* New mode handshake complete *)
            eTargetMode := G_sysRequestedMode;

            (* Check homing requirements for operational modes *)
            IF FN_IsOperationalMode(eTargetMode) AND
               (G_flagAbsHomeRequired OR G_flagEOTHomeRequired) THEN
                G_sysActiveFault := E_FaultCode#FAULT_HOMING_REQ;
                G_sysCurrentState := E_SystemState#ST_FAULT;
            ELSE
                (* Transition directly to new mode (drive already enabled) *)
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
            (* Handshake timeout *)
            G_sysActiveFault := E_FaultCode#FAULT_HANDSHAKE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - DEACTIVATION
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE THEN
        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;  (* Engage brake *)
            tBrakeTimer(IN := FALSE);
            fbDirectControl(Axis := G_sysAxis, Enable := FALSE);
        END_IF;

        (* Wait for brake engage delay *)
        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;
            G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_DISABLE THEN
        IF bEnterState THEN
            fbPower(Axis := G_sysAxis, Enable := FALSE);
        END_IF;

        fbPower(Axis := G_sysAxis, Enable := FALSE);

        IF NOT fbPower.Status THEN
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
           - Fast recovery: Handshake before timeout -> E_SystemState#ST_BRAKE_HOLD (drive stays on)
           - Slow recovery: Timeout -> E_SystemState#ST_FAULT_IDLE (drive off, brake on)
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            (* DO NOT engage brake - keep it disengaged for controlled stop *)
            (* G_doBrakeDisengage := FALSE;  -- REMOVED per design *)

            (* Stop motion (controlled deceleration via MC_Stop) *)
            fbStop(Axis := G_sysAxis, Execute := TRUE);
            fbDirectControl(Axis := G_sysAxis, Enable := FALSE);

            (* Signal fault *)
            G_doInMotion := FALSE;

            (* Record fault information *)
            G_sysLastFaultCode := G_sysActiveFault;
            (* Note: G_sysLastFaultState is set above, before G_sysPreviousState is overwritten *)
            G_sysFaultCount := G_sysFaultCount + 1;

            (* Start timeout timer for slow recovery path *)
            tStateTimer(IN := FALSE);
        END_IF;

        (* Continue halt until done *)
        fbStop(Axis := G_sysAxis, Execute := NOT fbStop.Done);

        (* Disable mode handshake in fault state *)
        fbHandshake(Enable := FALSE);

        (* Update fault reset handler - requires master to mirror fault code *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbFaultReset(
            Enable := TRUE,
            FaultReset := bFilteredFaultReset,
            MotionEnable := bFilteredMotionEnable,
            InFaultState := TRUE,
            BitsStable := fbModeDecoder.ModeStable,
            ActiveFaultCode := G_sysActiveFault,
            MasterFaultCode := FN_BitsToFaultCode(bFilteredModeBit0, bFilteredModeBit1, bFilteredModeBit2)
        );

        (* Timeout for slow recovery path *)
        tStateTimer(IN := TRUE, PT := G_cfgFaultIdleTimeout);

        (* Check for fault reset *)
        IF fbFaultReset.FaultResetValid THEN
            (* FAST RECOVERY PATH: Master responded before timeout *)
            (* Clear fault and go directly to brake hold - drive stays enabled *)
            (* Skip MC_Reset - not needed for most faults *)
            G_sysActiveFault := E_FaultCode#FAULT_NONE;
            fbStop(Axis := G_sysAxis, Execute := FALSE);
            G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;

        ELSIF tStateTimer.Q THEN
            (* SLOW RECOVERY PATH: Timeout reached *)
            (* Transition to fault-idle: disable drive, engage brake *)
            fbStop(Axis := G_sysAxis, Execute := FALSE);
            G_sysCurrentState := E_SystemState#ST_FAULT_IDLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_RECOVERY THEN
        (* ----------------------------------------------------------------
           LEGACY STATE - Kept for MC_Reset if needed
           Currently bypassed by fast recovery path in ST_FAULT
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            fbReset(Axis := G_sysAxis, Execute := TRUE);
            fbStop(Axis := G_sysAxis, Execute := FALSE);
        END_IF;

        fbReset(Axis := G_sysAxis, Execute := TRUE);

        IF fbReset.Done THEN
            (* Reset complete - go to fault idle *)
            G_sysActiveFault := E_FaultCode#FAULT_NONE;
            fbReset(Execute := FALSE);
            G_sysCurrentState := E_SystemState#ST_FAULT_IDLE;

        ELSIF fbReset.Error THEN
            (* Cannot recover - return to fault *)
            fbReset(Execute := FALSE);
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
        (* ----------------------------------------------------------------
           FAULT IDLE STATE - Drive disabled, brake engaged

           Entered after:
           - E_SystemState#ST_FAULT timeout (slow recovery)
           - E_SystemState#ST_FAULT_RECOVERY completion

           Awaits fault reset handshake to return to normal E_SystemState#ST_IDLE.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            (* Engage brake NOW (after fault is stabilized) *)
            G_doBrakeDisengage := FALSE;
            tBrakeTimer(IN := FALSE);

            (* Clear motion outputs *)
            G_doInMotion := FALSE;
            fbStop(Axis := G_sysAxis, Execute := FALSE);
        END_IF;

        (* Wait for brake engage delay before disabling drive *)
        tBrakeTimer(IN := TRUE, PT := G_cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;

            (* Disable drive power *)
            fbPower(Axis := G_sysAxis, Enable := FALSE);
            IF NOT fbPower.Status THEN
                G_sysDriveEnabled := FALSE;
            END_IF;
        END_IF;

        (* Disable mode handshake in fault idle state *)
        fbHandshake(Enable := FALSE);

        (* Update fault reset handler *)
        fbFaultReset(
            Enable := TRUE,
            FaultReset := bFilteredFaultReset,
            MotionEnable := bFilteredMotionEnable,
            InFaultState := (G_sysActiveFault <> E_FaultCode#FAULT_NONE),
            BitsStable := fbModeDecoder.ModeStable,
            ActiveFaultCode := G_sysLastFaultCode,
            MasterFaultCode := FN_BitsToFaultCode(bFilteredModeBit0, bFilteredModeBit1, bFilteredModeBit2)
        );

        (* Check for fault reset to return to normal idle *)
        IF fbFaultReset.FaultResetValid THEN
            (* Fault reset complete - return to normal idle *)
            G_sysActiveFault := E_FaultCode#FAULT_NONE;
            G_sysCurrentState := E_SystemState#ST_IDLE;
        END_IF;

END_IF;


(* --------------------------------------------------------------------------
   Safety Monitoring
   Runs every scan after state machine, before output update
   -------------------------------------------------------------------------- *)

(* Piston exit guard - detects dangerous pressure-overcoming-motor condition *)
(* Mode-independent: monitors actual position, velocity, and torque *)
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
G_sysExitGuardActive := fbPistonExitGuard.GuardActive;
G_sysInCylinderZone := NOT fbPistonExitGuard.InDangerZone;

(* Safety monitor - detects faults from various sources *)
fbSafetyMonitor(
    Enable := TRUE,
    LimitRetractActive := G_sysLimitRetractActive,
    LimitHomeActive := G_sysLimitHomeActive,
    ActualPosition := G_sysActualPosition,
    SoftLimitMin := G_cfgPosSoftLimitMin,
    SoftLimitMax := G_cfgPosSoftLimitMax,
    DriveReady := G_sysDriveReady,
    DriveFault := G_sysDriveFault,
    EncoderValid := G_sysEncoderValid,
    EncoderBatteryOK := G_sysEncoderBatteryOK,
    CurrentState := G_sysCurrentState,
    CurrentMode := G_sysConfirmedMode,
    ExitGuardTriggered := fbPistonExitGuard.FaultTriggered,
    AbsHomeRequired := G_flagAbsHomeRequired
);

(* Trigger fault if safety monitor detects issue (unless already in fault) *)
IF fbSafetyMonitor.FaultDetected AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT) AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT_RECOVERY) AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT_IDLE) AND
   (G_sysCurrentState <> E_SystemState#ST_INIT) AND
   (G_sysCurrentState <> E_SystemState#ST_ENCODER_CHECK) THEN
    G_sysActiveFault := fbSafetyMonitor.FaultCode;
    G_sysCurrentState := E_SystemState#ST_FAULT;
END_IF;


(* --------------------------------------------------------------------------
   Update Outputs
   -------------------------------------------------------------------------- *)

(* Update output multiplexer - selects mode confirm or fault code for DO0-DO2 *)
fbOutputMux(
    Enable := TRUE,
    FaultActive := (G_sysCurrentState = E_SystemState#ST_FAULT) OR (G_sysCurrentState = E_SystemState#ST_FAULT_RECOVERY) OR (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE),
    FaultCode := G_sysActiveFault,
    ConfirmedMode := fbHandshake.ConfirmedMode
);

G_doModeConfBit0 := fbOutputMux.DO0_Output;
G_doModeConfBit1 := fbOutputMux.DO1_Output;
G_doModeConfBit2 := fbOutputMux.DO2_Output;
G_doFaultActive := fbOutputMux.FaultActiveOutput;

(* Update position output - apply EOT offset for master coordinates *)
(* Master sees position in master coordinates: Master = Slave + Offset *)
(* Uses consolidated two-stage mapping config - transition point shared between stages *)
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
G_aoPositionOutput := fbPosOutput.RawAnalogOutput;

(* Increment cycle counter *)
G_sysCycleCount := G_sysCycleCount + 1;
