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
       fbHalt              : MC_Halt
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
       rTrigFaultReset     : R_TRIG
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
    sysCurrentState := ST_INIT;
    sysPreviousState := ST_INIT;

    (* Initialize homing flags *)
    flagEOTHomeRequired := TRUE;    (* Always required on power-up *)
    flagAbsHomeRequired := FALSE;   (* Will be set by encoder check *)

    (* Initialize outputs to safe state *)
    doBrakeDisengage := FALSE;      (* Brake engaged *)
    doFaultActive := FALSE;
    doPerformanceStatus := FALSE;
    doHomingComplete := FALSE;
    doInMotion := FALSE;

    (* Calculate analog filter alpha: alpha = dt / (tau + dt)
       dt = 1ms (fixed task cycle time from MP2600iec hardware config)
       tau = cfgAnalogFilterTimeConst *)
    IF cfgAnalogFilterTimeConst > T#0MS THEN
        rAnalogFilterAlpha := 0.001 / ((TIME_TO_LREAL(cfgAnalogFilterTimeConst) / 1000.0) + 0.001);
    ELSE
        rAnalogFilterAlpha := 1.0;  (* No filtering if time constant is zero *)
    END_IF;
    rAnalogFilterAlpha := LIMIT(0.0, rAnalogFilterAlpha, 1.0);
END_IF;


(* --------------------------------------------------------------------------
   Filter Digital Inputs (Debouncing)
   -------------------------------------------------------------------------- *)

(* Mode bits - standard debounce *)
fbFilterModeBit0(RawInput := diModeBit0, DebounceTime := cfgDebounceTimeMode);
fbFilterModeBit1(RawInput := diModeBit1, DebounceTime := cfgDebounceTimeMode);
fbFilterModeBit2(RawInput := diModeBit2, DebounceTime := cfgDebounceTimeMode);
bFilteredModeBit0 := fbFilterModeBit0.FilteredOutput;
bFilteredModeBit1 := fbFilterModeBit1.FilteredOutput;
bFilteredModeBit2 := fbFilterModeBit2.FilteredOutput;

(* Motion enable - shorter debounce for responsiveness *)
fbFilterMotionEnable(RawInput := diMotionEnable, DebounceTime := cfgDebounceTimeMotion);
bFilteredMotionEnable := fbFilterMotionEnable.FilteredOutput;

(* Limit switches - short debounce for responsiveness *)
fbFilterLimitRetract(RawInput := diLimitRetract, DebounceTime := cfgDebounceTimeLimitStd);
fbFilterLimitHome(RawInput := diLimitHome, DebounceTime := cfgDebounceTimeLimitHome);
bFilteredLimitRetract := fbFilterLimitRetract.FilteredOutput;
bFilteredLimitHome := fbFilterLimitHome.FilteredOutput;

(* Fault reset *)
fbFilterFaultReset(RawInput := diFaultReset, DebounceTime := cfgDebounceTimeFault);
bFilteredFaultReset := fbFilterFaultReset.FilteredOutput;


(* --------------------------------------------------------------------------
   Filter Analog Input (Median + Low-Pass)
   -------------------------------------------------------------------------- *)

fbFilterAnalog(
    Enable := TRUE,
    RawInput := aiReference,
    Alpha := rAnalogFilterAlpha,
    MedianFilterSize := cfgAnalogMedianSize
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
sysRequestedMode := fbModeDecoder.DecodedMode;

(* Edge detection on filtered motion enable *)
rTrigMotionEnable(CLK := bFilteredMotionEnable);
fTrigMotionEnable(CLK := bFilteredMotionEnable);

(* Edge detection on filtered fault reset *)
rTrigFaultReset(CLK := bFilteredFaultReset);

(* Read actual position, velocity, torque from drive *)
fbReadActualPos(Axis := sysAxis, Enable := TRUE);
fbReadActualVel(Axis := sysAxis, Enable := TRUE);
fbReadActualTorque(Axis := sysAxis, Enable := TRUE);

IF fbReadActualPos.Valid THEN
    sysActualPosition := fbReadActualPos.Position;
END_IF;
IF fbReadActualVel.Valid THEN
    sysActualVelocity := fbReadActualVel.Velocity;
END_IF;
IF fbReadActualTorque.Valid THEN
    sysActualTorque := fbReadActualTorque.Torque;
END_IF;

(* Update limit switch status from filtered inputs (PNP NC: FALSE = triggered) *)
sysLimitRetractActive := NOT bFilteredLimitRetract;
sysLimitHomeActive := NOT bFilteredLimitHome;

(* Process filtered analog input - uses two-stage mapping for position reference *)
fbAnalogProc(
    Enable := TRUE,
    RawAnalogInput := nFilteredAnalog,
    CurrentMode := sysRequestedMode,
    (* Two-stage position mapping (same config as position output) *)
    Stage1PosMin := cfgPosMapStage1PosMin,
    Stage1PosMax := cfgPosMapTransitionPos,
    Stage1VoltMin := cfgPosMapStage1VoltMin,
    Stage1VoltMax := cfgPosMapTransitionVolt,
    Stage2PosMin := cfgPosMapTransitionPos,
    Stage2PosMax := cfgPosMapStage2PosMax,
    Stage2VoltMin := cfgPosMapTransitionVolt,
    Stage2VoltMax := cfgPosMapStage2VoltMax,
    (* Linear velocity and torque scaling *)
    VelMin := cfgAnalogVelMin,
    VelMax := cfgAnalogVelMax,
    TorqueMin := cfgAnalogTorqueMin,
    TorqueMax := cfgAnalogTorqueMax
);


(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

(* Detect state entry *)
bEnterState := (sysCurrentState <> sysPreviousState);
IF bEnterState THEN
    (* Capture fault origin before sysPreviousState is overwritten *)
    IF sysCurrentState = ST_FAULT THEN
        sysLastFaultState := sysPreviousState;
    END_IF;
    sysPreviousState := sysCurrentState;
    tStateTimer(IN := FALSE);   (* Reset state timer on entry *)
END_IF;

CASE sysCurrentState OF

    (* ======================================================================
       INITIALIZATION STATES
       ====================================================================== *)

    ST_INIT:
        (* Power-on initialization *)
        IF bEnterState THEN
            (* Ensure safe output state *)
            doBrakeDisengage := FALSE;
            doFaultActive := FALSE;
            fbPower(Axis := sysAxis, Enable := FALSE);
        END_IF;

        (* Proceed to encoder check *)
        sysCurrentState := ST_ENCODER_CHECK;


    ST_ENCODER_CHECK:
        (* Validate absolute encoder status using AbsolutePositionManager *)
        IF bEnterState THEN
            fbEncoderManager(Enable := FALSE);  (* Reset *)
        END_IF;

        (* Run encoder check *)
        fbEncoderManager(
            Enable := TRUE,
            Axis := sysAxis,
            CheckOnStartup := TRUE
        );

        IF fbEncoderManager.CheckComplete THEN
            (* Check complete - read results *)
            sysEncoderValid := fbEncoderManager.EncoderValid;
            sysEncoderBatteryOK := fbEncoderManager.BatteryOK;

            IF sysEncoderValid THEN
                (* Encoder valid - proceed to idle *)
                flagAbsHomeRequired := FALSE;
                sysCurrentState := ST_IDLE;
            ELSE
                (* Encoder invalid - set flag and proceed *)
                sysCurrentState := ST_REQUIRE_ABS_HOME;
            END_IF;

        ELSIF fbEncoderManager.Error THEN
            (* Encoder check error - assume invalid, require homing *)
            sysEncoderValid := FALSE;
            sysCurrentState := ST_REQUIRE_ABS_HOME;
        END_IF;


    ST_REQUIRE_ABS_HOME:
        (* Set absolute home requirement flag *)
        IF bEnterState THEN
            flagAbsHomeRequired := TRUE;
        END_IF;

        (* Proceed to idle *)
        sysCurrentState := ST_IDLE;


    (* ======================================================================
       IDLE STATE
       ====================================================================== *)

    ST_IDLE:
        IF bEnterState THEN
            (* Ensure safe state *)
            doBrakeDisengage := FALSE;  (* Brake engaged *)
            sysDriveEnabled := FALSE;
            sysDriveReady := FALSE;
            fbPower(Axis := sysAxis, Enable := FALSE);
            (* NOTE: sysConfirmedMode is used by fbHandshake to set DO0-2 output bits.
               It represents the mode the slave is confirming to the master. *)
            sysConfirmedMode := MODE_IDLE;
        END_IF;

        (* Update handshake manager - it handles mode confirmation output *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            FaultReset := bFilteredFaultReset,
            InFaultState := FALSE,
            HandshakeTimeout := cfgHandshakeTimeout
        );

        (* Wait for handshake to complete before transitioning *)
        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := sysRequestedMode;

            (* Check if this is an operational mode requiring homing *)
            IF IsOperationalMode(eTargetMode) THEN
                (* Check homing requirements *)
                IF flagAbsHomeRequired OR flagEOTHomeRequired THEN
                    (* Homing required but not satisfied - enter fault *)
                    sysActiveFault := FAULT_HOMING_REQ;
                    sysCurrentState := ST_FAULT;
                ELSE
                    (* Requirements met - proceed to drive enable *)
                    sysCurrentState := ST_DRIVE_ENABLE;
                END_IF;

            ELSIF IsHomingMode(eTargetMode) THEN
                (* Homing modes - check ordering requirements *)
                IF (eTargetMode = MODE_HOME_EOT) AND flagAbsHomeRequired THEN
                    (* EOT homing requires limit switch homing first if encoder invalid *)
                    sysActiveFault := FAULT_HOMING_REQ;
                    sysCurrentState := ST_FAULT;
                ELSE
                    (* Homing mode allowed - proceed *)
                    sysCurrentState := ST_DRIVE_ENABLE;
                END_IF;

            ELSE
                (* Unknown mode - stay in idle *)
                ;
            END_IF;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - ACTIVATION
       ====================================================================== *)

    ST_DRIVE_ENABLE:
        IF bEnterState THEN
            tDriveReadyTimer(IN := FALSE);
        END_IF;

        (* Enable drive power and wait for ready *)
        fbPower(Axis := sysAxis, Enable := TRUE,
                EnablePositive := TRUE, EnableNegative := TRUE);

        (* Wait for drive ready with timeout *)
        tDriveReadyTimer(IN := TRUE, PT := cfgDriveReadyTimeout);

        IF fbPower.Status THEN
            (* Drive is ready *)
            sysDriveEnabled := TRUE;
            sysDriveReady := TRUE;
            sysCurrentState := ST_BRAKE_RELEASE;

        ELSIF fbPower.Error OR tDriveReadyTimer.Q THEN
            (* Drive error or timeout *)
            sysActiveFault := FAULT_DRIVE;
            sysCurrentState := ST_FAULT;
        END_IF;


    ST_BRAKE_RELEASE:
        IF bEnterState THEN
            doBrakeDisengage := TRUE;   (* Release brake *)
            tBrakeTimer(IN := FALSE);
        END_IF;

        (* Wait for brake release delay *)
        tBrakeTimer(IN := TRUE, PT := cfgBrakeDisengageDelay);

        IF tBrakeTimer.Q THEN
            sysBrakeEngaged := FALSE;

            (* Transition to target operational state *)
            CASE eTargetMode OF
                MODE_BRAKE_HOLD:    sysCurrentState := ST_BRAKE_HOLD;
                MODE_POSITION:      sysCurrentState := ST_POSITION_CTRL;
                MODE_VELOCITY:      sysCurrentState := ST_VELOCITY_CTRL;
                MODE_TORQUE:        sysCurrentState := ST_TORQUE_CTRL;
                MODE_GO_HOME:       sysCurrentState := ST_GO_HOME;
                MODE_HOME_LIMIT:    sysCurrentState := ST_HOME_LIMIT;
                MODE_HOME_EOT:      sysCurrentState := ST_HOME_EOT;
            ELSE
                (* Unknown mode - brake is released, hold position until handshake *)
                sysCurrentState := ST_HOLD_POSITION;
            END_CASE;
        END_IF;


    (* ======================================================================
       OPERATIONAL STATES
       ====================================================================== *)

    ST_BRAKE_HOLD:
        IF bEnterState THEN
            doBrakeDisengage := FALSE;  (* Engage brake *)
            sysConfirmedMode := MODE_BRAKE_HOLD;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        doInMotion := FALSE;

        (* Update handshake manager for mode changes *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            FaultReset := bFilteredFaultReset,
            InFaultState := FALSE,
            HandshakeTimeout := cfgHandshakeTimeout
        );

        (* Allow mode changes when handshake completes *)
        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := sysRequestedMode;

            (* Check homing requirements for operational modes *)
            IF IsOperationalMode(eTargetMode) AND
               (flagAbsHomeRequired OR flagEOTHomeRequired) THEN
                sysActiveFault := FAULT_HOMING_REQ;
                sysCurrentState := ST_FAULT;
            ELSE
                CASE eTargetMode OF
                    MODE_IDLE:
                        (* Return to idle - disable drive *)
                        sysCurrentState := ST_DRIVE_DISABLE;

                    MODE_BRAKE_HOLD:
                        (* Stay in brake hold - do nothing *)
                        ;

                    MODE_POSITION, MODE_VELOCITY, MODE_TORQUE,
                    MODE_GO_HOME, MODE_HOME_LIMIT, MODE_HOME_EOT:
                        (* Other operational modes - need to release brake first *)
                        sysCurrentState := ST_BRAKE_RELEASE;
                END_CASE;
            END_IF;
        END_IF;

        (* Note: ST_BRAKE_HOLD does NOT transition to ST_HOLD_POSITION on motion enable drop.
           It is safer to keep the brake engaged. Mode changes are handled via handshake above. *)


    ST_POSITION_CTRL:
        IF bEnterState THEN
            sysConfirmedMode := MODE_POSITION;
            (* Configure position mode limits *)
            stCurrentLimits.PosMax := cfgPosSoftLimitMax;
            stCurrentLimits.PosMin := cfgPosSoftLimitMin;
            stCurrentLimits.VelMax := cfgVelLimitNormal;
            stCurrentLimits.TorqueMax := cfgTorqueLimitNormal;
        END_IF;

        (* Get position reference from analog input and apply EOT offset *)
        (* Master commands in master coordinates, convert to slave coordinates *)
        (* Slave = Master - Offset *)
        sysCommandedPosition := fbAnalogProc.ScaledPosition - posEOTOffset;

        (* Apply position limits - additional safety layer *)
        sysCommandedPosition := LIMIT(stCurrentLimits.PosMin,
                                      sysCommandedPosition,
                                      stCurrentLimits.PosMax);

        (* Execute position control via Y_DirectControl *)
        fbDirectControl(Axis := sysAxis,
                        Enable := TRUE,
                        Mode := 1,  (* Position mode *)
                        Position := sysCommandedPosition,
                        Velocity := stCurrentLimits.VelMax,
                        Torque := stCurrentLimits.TorqueMax);

        doInMotion := ABS(sysActualVelocity) > cfgInMotionVelThreshold;

        (* Check for motion enable drop *)
        IF fTrigMotionEnable.Q THEN
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    ST_VELOCITY_CTRL:
        IF bEnterState THEN
            sysConfirmedMode := MODE_VELOCITY;
            stCurrentLimits.VelMax := cfgVelLimitMax;
            stCurrentLimits.TorqueMax := cfgTorqueLimitNormal;
        END_IF;

        (* Get velocity reference from analog input *)
        sysCommandedVelocity := fbAnalogProc.ScaledVelocity;

        (* Apply velocity limits *)
        sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      sysCommandedVelocity,
                                      stCurrentLimits.VelMax);

        (* Execute velocity control *)
        fbDirectControl(Axis := sysAxis,
                        Enable := TRUE,
                        Mode := 2,  (* Velocity mode *)
                        Velocity := sysCommandedVelocity,
                        Torque := stCurrentLimits.TorqueMax);

        doInMotion := ABS(sysActualVelocity) > cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    ST_TORQUE_CTRL:
        IF bEnterState THEN
            sysConfirmedMode := MODE_TORQUE;
            stCurrentLimits.TorqueMax := cfgTorqueLimitMax;
            stCurrentLimits.VelMax := cfgVelLimitMax;
        END_IF;

        (* Get torque reference from analog input *)
        sysCommandedTorque := fbAnalogProc.ScaledTorque;

        (* Apply torque limits *)
        sysCommandedTorque := LIMIT(-stCurrentLimits.TorqueMax,
                                    sysCommandedTorque,
                                    stCurrentLimits.TorqueMax);

        (* Execute torque control *)
        fbDirectControl(Axis := sysAxis,
                        Enable := TRUE,
                        Mode := 3,  (* Torque mode *)
                        Velocity := stCurrentLimits.VelMax,  (* As limit *)
                        Torque := sysCommandedTorque);

        doInMotion := ABS(sysActualVelocity) > cfgInMotionVelThreshold;

        IF fTrigMotionEnable.Q THEN
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    ST_GO_HOME:
        IF bEnterState THEN
            sysConfirmedMode := MODE_GO_HOME;
            fbGoHome(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Use FB_GoHome for homing requirement checking and move *)
        (* Apply offset: Slave position = Master position - Offset *)
        fbGoHome(
            Enable := TRUE,
            Execute := TRUE,
            Axis := sysAxis,
            AbsHomeRequired := flagAbsHomeRequired,
            EOTHomeRequired := flagEOTHomeRequired,
            HomePosition := cfgGoHomePosition - posEOTOffset,
            HomeVelocity := cfgGoHomeVelocity,
            Abort := fTrigMotionEnable.Q
        );

        doInMotion := fbGoHome.InMotion;

        IF fbGoHome.RedirectToHoming THEN
            (* Need limit switch homing first - redirect *)
            fbGoHome(Enable := FALSE);
            sysCurrentState := ST_HOME_LIMIT;

        ELSIF fbGoHome.Done THEN
            (* At home position - stay here until mode change *)
            doInMotion := FALSE;

        ELSIF fbGoHome.Error THEN
            (* Error during move *)
            fbGoHome(Enable := FALSE);
            sysActiveFault := FAULT_DRIVE;
            sysCurrentState := ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            (* Note: ELSIF ensures RedirectToHoming is FALSE at this point *)
            fbGoHome(Enable := FALSE);
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    ST_HOME_LIMIT:
        (* Mode 110 - Home to Limit Switch
           Uses FB_HomeLimit to encapsulate the complete homing sequence:
           approach, over-travel handling, detect, backoff, and set reference *)
        IF bEnterState THEN
            sysConfirmedMode := MODE_HOME_LIMIT;
            flagHomingInProgress := TRUE;
            fbHomeLimit(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Call homing FB *)
        fbHomeLimit(
            Enable := TRUE,
            Execute := TRUE,
            Axis := sysAxis,
            ApproachVelocity := cfgHomeLimApproachVel,
            BackoffVelocity := cfgVelHomingSlow,
            SetPosition := cfgHomeLimSetPosition,
            Timeout := cfgHomingTimeout,
            LimitHomeActive := sysLimitHomeActive,
            LimitRetractActive := sysLimitRetractActive,
            Abort := fTrigMotionEnable.Q
        );

        doInMotion := fbHomeLimit.InMotion;

        IF fbHomeLimit.Done THEN
            (* Homing complete - clear requirement and restore encoder validity *)
            fbHomeLimit(Enable := FALSE);
            flagAbsHomeRequired := FALSE;
            sysEncoderValid := TRUE;
            flagHomingComplete := TRUE;
            sysCurrentState := ST_HOME_COMPLETE;

        ELSIF fbHomeLimit.Error THEN
            (* Homing failed *)
            fbHomeLimit(Enable := FALSE);
            sysActiveFault := FAULT_LIMIT_SWITCH;
            sysCurrentState := ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            fbHomeLimit(Enable := FALSE);
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
    ST_HOME_LIM_APPROACH,
    ST_HOME_LIM_DETECT,
    ST_HOME_LIM_BACKOFF,
    ST_HOME_LIM_SETREF:
        (* These sub-states are now handled internally by FB_HomeLimit.
           If we get here, redirect back to ST_HOME_LIMIT *)
        sysCurrentState := ST_HOME_LIMIT;


    ST_HOME_EOT:
        (* Mode 111 - Home to End-of-Travel
           Uses FB_HomeEOT to encapsulate the complete homing sequence:
           check requirements, fast approach, slow approach, stall detect, offset calc *)
        IF bEnterState THEN
            sysConfirmedMode := MODE_HOME_EOT;
            flagHomingInProgress := TRUE;
            fbHomeEOT(Enable := FALSE);  (* Reset FB *)
        END_IF;

        (* Call homing FB *)
        fbHomeEOT(
            Enable := TRUE,
            Execute := TRUE,
            Axis := sysAxis,
            AbsHomeRequired := flagAbsHomeRequired,
            FastVelocity := cfgHomeEOTFastVel,
            SlowVelocity := cfgHomeEOTSlowVel,
            TorqueLimit := cfgTorqueHomingLimit,
            TorqueThreshold := cfgHomeEOTTorqueThresh,
            ApproachDistance := cfgHomeEOTApproachDist,
            ExpectedEOTPosition := cfgHomeEOTSetPosition,
            StallDetectTime := cfgStallDetectTime,
            Timeout := cfgHomingTimeout,
            ActualPosition := sysActualPosition,
            ActualVelocity := sysActualVelocity,
            ActualTorque := sysActualTorque,
            Abort := fTrigMotionEnable.Q
        );

        doInMotion := fbHomeEOT.InMotion;

        IF fbHomeEOT.RequiresLimitHome THEN
            (* Must do limit switch homing before EOT homing *)
            fbHomeEOT(Enable := FALSE);
            sysActiveFault := FAULT_HOMING_REQ;
            sysCurrentState := ST_FAULT;

        ELSIF fbHomeEOT.Done THEN
            (* EOT homing complete - get results and clear requirement *)
            posActualEOT := fbHomeEOT.EOTPosition;
            posEOTOffset := fbHomeEOT.EOTOffset;
            fbHomeEOT(Enable := FALSE);
            flagEOTHomeRequired := FALSE;
            flagHomingComplete := TRUE;
            sysCurrentState := ST_HOME_COMPLETE;

        ELSIF fbHomeEOT.Error THEN
            (* Homing failed *)
            fbHomeEOT(Enable := FALSE);
            sysActiveFault := FAULT_POSITION;
            sysCurrentState := ST_FAULT;

        ELSIF fTrigMotionEnable.Q THEN
            (* Motion enable dropped - abort and hold *)
            fbHomeEOT(Enable := FALSE);
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
    ST_HOME_EOT_FAST,
    ST_HOME_EOT_SLOW,
    ST_HOME_EOT_DETECT,
    ST_HOME_EOT_SETREF:
        (* These sub-states are now handled internally by FB_HomeEOT.
           If we get here, redirect back to ST_HOME_EOT *)
        sysCurrentState := ST_HOME_EOT;


    ST_HOME_COMPLETE:
        IF bEnterState THEN
            flagHomingInProgress := FALSE;
            doHomingComplete := TRUE;
        END_IF;

        doInMotion := FALSE;

        IF fTrigMotionEnable.Q THEN
            doHomingComplete := FALSE;
            sysCurrentState := ST_HOLD_POSITION;
        END_IF;


    (* ======================================================================
       TRANSITION STATE - INTER-MODE
       ====================================================================== *)

    ST_HOLD_POSITION:
        IF bEnterState THEN
            (* IMPORTANT: Order matters here!
               1. fbHalt FIRST - initiates controlled deceleration and transitions axis
                  to PLCopen "Standstill" state which maintains active position control
               2. fbDirectControl SECOND - releases Y_DirectControl (Yaskawa-specific)

               If reversed, Y_DirectControl would release servo reference while still
               moving, potentially causing a drive fault. MC_Halt must take over
               position control BEFORE Y_DirectControl releases it. *)
            fbHalt(Axis := sysAxis, Execute := TRUE);
            fbDirectControl(Axis := sysAxis, Enable := FALSE);
            sysHalted := FALSE;
            tHandshakeTimer(IN := FALSE);
        END_IF;

        (* Wait for halt to complete *)
        IF NOT sysHalted THEN
            fbHalt(Axis := sysAxis, Execute := TRUE);
            IF fbHalt.Done THEN
                sysHalted := TRUE;
                fbHalt(Execute := FALSE);
            END_IF;
        END_IF;

        doInMotion := NOT sysHalted;

        (* Update handshake manager *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            FaultReset := bFilteredFaultReset,
            InFaultState := FALSE,
            HandshakeTimeout := cfgHandshakeTimeout
        );

        (* Wait for new mode handshake *)
        tHandshakeTimer(IN := TRUE, PT := cfgHandshakeTimeout);

        IF fbHandshake.HandshakeComplete THEN
            (* New mode handshake complete *)
            eTargetMode := sysRequestedMode;

            (* Check homing requirements for operational modes *)
            IF IsOperationalMode(eTargetMode) AND
               (flagAbsHomeRequired OR flagEOTHomeRequired) THEN
                sysActiveFault := FAULT_HOMING_REQ;
                sysCurrentState := ST_FAULT;
            ELSE
                (* Transition directly to new mode (drive already enabled) *)
                CASE eTargetMode OF
                    MODE_BRAKE_HOLD:    sysCurrentState := ST_BRAKE_HOLD;
                    MODE_POSITION:      sysCurrentState := ST_POSITION_CTRL;
                    MODE_VELOCITY:      sysCurrentState := ST_VELOCITY_CTRL;
                    MODE_TORQUE:        sysCurrentState := ST_TORQUE_CTRL;
                    MODE_GO_HOME:       sysCurrentState := ST_GO_HOME;
                    MODE_HOME_LIMIT:    sysCurrentState := ST_HOME_LIMIT;
                    MODE_HOME_EOT:      sysCurrentState := ST_HOME_EOT;
                    MODE_IDLE:          sysCurrentState := ST_BRAKE_ENGAGE;
                END_CASE;
            END_IF;

        ELSIF tHandshakeTimer.Q THEN
            (* Handshake timeout *)
            sysActiveFault := FAULT_HANDSHAKE;
            sysCurrentState := ST_FAULT;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - DEACTIVATION
       ====================================================================== *)

    ST_BRAKE_ENGAGE:
        IF bEnterState THEN
            doBrakeDisengage := FALSE;  (* Engage brake *)
            tBrakeTimer(IN := FALSE);
            fbDirectControl(Axis := sysAxis, Enable := FALSE);
        END_IF;

        (* Wait for brake engage delay *)
        tBrakeTimer(IN := TRUE, PT := cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            sysBrakeEngaged := TRUE;
            sysCurrentState := ST_DRIVE_DISABLE;
        END_IF;


    ST_DRIVE_DISABLE:
        IF bEnterState THEN
            fbPower(Axis := sysAxis, Enable := FALSE);
        END_IF;

        fbPower(Axis := sysAxis, Enable := FALSE);

        IF NOT fbPower.Status THEN
            sysDriveEnabled := FALSE;
            sysDriveReady := FALSE;
            sysCurrentState := ST_IDLE;
        END_IF;


    (* ======================================================================
       FAULT STATES
       ====================================================================== *)

    ST_FAULT:
        (* ----------------------------------------------------------------
           FAULT STATE - Active fault handling

           NO brake engagement here - drive holds position via MC_Halt.
           Two exit paths:
           - Fast recovery: Handshake before timeout -> ST_BRAKE_HOLD (drive stays on)
           - Slow recovery: Timeout -> ST_FAULT_IDLE (drive off, brake on)
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            (* DO NOT engage brake - keep it disengaged for controlled stop *)
            (* doBrakeDisengage := FALSE;  -- REMOVED per design *)

            (* Halt motion (controlled stop, less aggressive than MC_Stop) *)
            fbHalt(Axis := sysAxis, Execute := TRUE);
            fbDirectControl(Axis := sysAxis, Enable := FALSE);

            (* Signal fault *)
            doFaultActive := TRUE;
            doInMotion := FALSE;

            (* Record fault information *)
            sysLastFaultCode := sysActiveFault;
            (* Note: sysLastFaultState is set above, before sysPreviousState is overwritten *)
            sysFaultCount := sysFaultCount + 1;

            (* Start timeout timer for slow recovery path *)
            tStateTimer(IN := FALSE);
        END_IF;

        (* Continue halt until done *)
        fbHalt(Axis := sysAxis, Execute := NOT fbHalt.Done);

        (* Update handshake for fault reset - requires master to mirror fault code *)
        (* NOTE: Uses filtered inputs for noise immunity *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            FaultReset := bFilteredFaultReset,
            InFaultState := TRUE,
            ActiveFaultCode := sysActiveFault,
            HandshakeTimeout := cfgHandshakeTimeout
        );

        (* Timeout for slow recovery path *)
        tStateTimer(IN := TRUE, PT := cfgFaultIdleTimeout);

        (* Check for fault reset handshake *)
        IF fbHandshake.FaultResetValid THEN
            (* FAST RECOVERY PATH: Master responded before timeout *)
            (* Clear fault and go directly to brake hold - drive stays enabled *)
            (* Skip MC_Reset - not needed for most faults *)
            sysActiveFault := FAULT_NONE;
            doFaultActive := FALSE;
            fbHalt(Axis := sysAxis, Execute := FALSE);
            sysCurrentState := ST_BRAKE_HOLD;

        ELSIF tStateTimer.Q THEN
            (* SLOW RECOVERY PATH: Timeout reached *)
            (* Transition to fault-idle: disable drive, engage brake *)
            fbHalt(Axis := sysAxis, Execute := FALSE);
            sysCurrentState := ST_FAULT_IDLE;
        END_IF;


    ST_FAULT_RECOVERY:
        (* ----------------------------------------------------------------
           LEGACY STATE - Kept for MC_Reset if needed
           Currently bypassed by fast recovery path in ST_FAULT
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            fbReset(Axis := sysAxis, Execute := TRUE);
            fbHalt(Axis := sysAxis, Execute := FALSE);
        END_IF;

        fbReset(Axis := sysAxis, Execute := TRUE);

        IF fbReset.Done THEN
            (* Reset complete - go to fault idle *)
            sysActiveFault := FAULT_NONE;
            doFaultActive := FALSE;
            fbReset(Execute := FALSE);
            sysCurrentState := ST_FAULT_IDLE;

        ELSIF fbReset.Error THEN
            (* Cannot recover - return to fault *)
            fbReset(Execute := FALSE);
            sysCurrentState := ST_FAULT;
        END_IF;


    ST_FAULT_IDLE:
        (* ----------------------------------------------------------------
           FAULT IDLE STATE - Drive disabled, brake engaged

           Entered after:
           - ST_FAULT timeout (slow recovery)
           - ST_FAULT_RECOVERY completion

           Awaits fault reset handshake to return to normal ST_IDLE.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            (* Engage brake NOW (after fault is stabilized) *)
            doBrakeDisengage := FALSE;
            tBrakeTimer(IN := FALSE);

            (* Clear motion outputs *)
            doInMotion := FALSE;
            fbHalt(Axis := sysAxis, Execute := FALSE);
        END_IF;

        (* Wait for brake engage delay before disabling drive *)
        tBrakeTimer(IN := TRUE, PT := cfgBrakeEngageDelay);

        IF tBrakeTimer.Q THEN
            sysBrakeEngaged := TRUE;

            (* Disable drive power *)
            fbPower(Axis := sysAxis, Enable := FALSE);
            IF NOT fbPower.Status THEN
                sysDriveEnabled := FALSE;
            END_IF;
        END_IF;

        (* Update handshake manager for fault reset *)
        fbHandshake(
            Enable := TRUE,
            RequestedMode := sysRequestedMode,
            ModeStable := fbModeDecoder.ModeStable,
            MotionEnable := bFilteredMotionEnable,
            FaultReset := bFilteredFaultReset,
            InFaultState := (sysActiveFault <> FAULT_NONE),
            ActiveFaultCode := sysLastFaultCode,
            HandshakeTimeout := cfgHandshakeTimeout
        );

        (* Check for fault reset handshake to return to normal idle *)
        IF fbHandshake.FaultResetValid THEN
            (* Fault reset complete - return to normal idle *)
            doFaultActive := FALSE;
            sysActiveFault := FAULT_NONE;
            sysCurrentState := ST_IDLE;
        END_IF;

END_CASE;


(* --------------------------------------------------------------------------
   Safety Monitoring
   Runs every scan after state machine, before output update
   -------------------------------------------------------------------------- *)

(* Piston exit guard - detects dangerous pressure-overcoming-motor condition *)
(* Mode-independent: monitors actual position, velocity, and torque *)
fbPistonExitGuard(
    Enable := TRUE,
    ActualPosition := sysActualPosition,
    ActualVelocity := sysActualVelocity,
    ActualTorque := sysActualTorque,
    SoftLimitMin := cfgPosSoftLimitMin,
    GuardMargin := cfgPosExitGuardMargin,
    VelocityThreshold := cfgInMotionVelThreshold,
    TorqueThreshold := cfgTorqueHomingThreshold
);
sysExitGuardActive := fbPistonExitGuard.GuardActive;
sysInCylinderZone := NOT fbPistonExitGuard.InDangerZone;

(* Safety monitor - detects faults from various sources *)
fbSafetyMonitor(
    Enable := TRUE,
    LimitRetractActive := sysLimitRetractActive,
    LimitHomeActive := sysLimitHomeActive,
    ActualPosition := sysActualPosition,
    SoftLimitMin := cfgPosSoftLimitMin,
    SoftLimitMax := cfgPosSoftLimitMax,
    DriveReady := sysDriveReady,
    DriveFault := sysDriveFault,
    DriveErrorID := 0,
    EncoderValid := sysEncoderValid,
    EncoderBatteryOK := sysEncoderBatteryOK,
    CurrentState := sysCurrentState,
    CurrentMode := sysConfirmedMode,
    ExitGuardTriggered := fbPistonExitGuard.FaultTriggered,
    AbsHomeRequired := flagAbsHomeRequired
);

(* Trigger fault if safety monitor detects issue (unless already in fault) *)
IF fbSafetyMonitor.FaultDetected AND
   (sysCurrentState <> ST_FAULT) AND
   (sysCurrentState <> ST_FAULT_RECOVERY) AND
   (sysCurrentState <> ST_FAULT_IDLE) AND
   (sysCurrentState <> ST_INIT) AND
   (sysCurrentState <> ST_ENCODER_CHECK) THEN
    sysActiveFault := fbSafetyMonitor.FaultCode;
    sysCurrentState := ST_FAULT;
END_IF;


(* --------------------------------------------------------------------------
   Update Outputs
   -------------------------------------------------------------------------- *)

(* Update output multiplexer - selects mode confirm or fault code for DO0-DO2 *)
fbOutputMux(
    Enable := TRUE,
    FaultActive := (sysCurrentState = ST_FAULT) OR (sysCurrentState = ST_FAULT_RECOVERY) OR (sysCurrentState = ST_FAULT_IDLE),
    FaultCode := sysActiveFault,
    ModeConfirmBit0 := fbHandshake.ConfirmBit0,
    ModeConfirmBit1 := fbHandshake.ConfirmBit1,
    ModeConfirmBit2 := fbHandshake.ConfirmBit2
);

doModeConfBit0 := fbOutputMux.DO0_Output;
doModeConfBit1 := fbOutputMux.DO1_Output;
doModeConfBit2 := fbOutputMux.DO2_Output;
doFaultActive := fbOutputMux.FaultActiveOutput;

(* Update position output - apply EOT offset for master coordinates *)
(* Master sees position in master coordinates: Master = Slave + Offset *)
(* Uses consolidated two-stage mapping config - transition point shared between stages *)
fbPosOutput(
    Enable := TRUE,
    ActualPosition := sysActualPosition + posEOTOffset,
    Stage1PosMin := cfgPosMapStage1PosMin,
    Stage1PosMax := cfgPosMapTransitionPos,
    Stage1VoltMin := cfgPosMapStage1VoltMin,
    Stage1VoltMax := cfgPosMapTransitionVolt,
    Stage2PosMin := cfgPosMapTransitionPos,
    Stage2PosMax := cfgPosMapStage2PosMax,
    Stage2VoltMin := cfgPosMapTransitionVolt,
    Stage2VoltMax := cfgPosMapStage2VoltMax
);
aoPositionOutput := fbPosOutput.RawAnalogOutput;

(* Increment cycle counter *)
sysCycleCount := sysCycleCount + 1;
