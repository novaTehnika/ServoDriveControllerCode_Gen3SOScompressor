(* ============================================================================
   GlobalVariables_Reference - Global Variable Reference Document

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This file documents all global variables that need to be configured
   in the MWiec GUI. It is a REFERENCE ONLY - variables are defined
   in the MotionWorks IEC Express global variable editor.

   Organized by functional group:
   1. I/O Mappings (from GVL_IO)
   2. Configurable Parameters (from GVL_Config)
   3. Position References (from GVL_Position)
   4. Runtime State Variables (from GVL_System)

   Units:
   - Position: millimeters (mm)
   - Velocity: mm/s
   - Torque: % of rated (100 = 100%, up to 800% peak)
   - Time: TIME type (T#xxxMS, T#xxxS)
   ============================================================================ *)


(* ============================================================================
   1. I/O MAPPINGS
   ============================================================================

   Note: Actual I/O addresses (%IX, %QX, %IW, %QW) must be verified against
   the hardware configuration in MotionWorksIEC Express.

   VAR_GLOBAL
     (* ========================================================================
        DIGITAL INPUTS (8 Available)
        PNP limit switches: TRUE = not triggered, FALSE = triggered (fail-safe)
        ======================================================================== *)

     (* Mode Command from Master (3-bit) *)
     G_diModeBit0          AT %IX0.0 : BOOL   (* Mode bit 0 - LSB *)
     G_diModeBit1          AT %IX0.1 : BOOL   (* Mode bit 1 *)
     G_diModeBit2          AT %IX0.2 : BOOL   (* Mode bit 2 - MSB *)

     (* Motion Control *)
     G_diMotionEnable      AT %IX0.3 : BOOL   (* Motion enable from master *)

     (* Limit Switches - Tolomatic #8100-9092, Solid State, NC, PNP *)
     (* TRUE = switch NOT triggered (normal), FALSE = triggered or wire broken *)
     G_diLimitRetract      AT %IX0.4 : BOOL   (* Retracted position limit switch *)
     G_diLimitHome         AT %IX0.5 : BOOL   (* Homing reference limit switch *)

     (* Fault Control *)
     G_diFaultReset        AT %IX0.6 : BOOL   (* Fault reset command from master *)

     (* Reserved *)
     G_diReserved          AT %IX0.7 : BOOL   (* Reserved for future use *)


     (* ========================================================================
        DIGITAL OUTPUTS (8 Available)
        ======================================================================== *)

     (* Mode Confirmation to Master (3-bit) / Fault Code when G_doFaultActive=TRUE *)
     G_doModeConfBit0      AT %QX0.0 : BOOL   (* Mode confirm bit 0 / Fault code bit 0 *)
     G_doModeConfBit1      AT %QX0.1 : BOOL   (* Mode confirm bit 1 / Fault code bit 1 *)
     G_doModeConfBit2      AT %QX0.2 : BOOL   (* Mode confirm bit 2 / Fault code bit 2 *)

     (* Brake Control *)
     (* Spring-engaged, electrically released: HIGH = brake disengaged (motion allowed) *)
     G_doBrakeDisengage    AT %QX0.3 : BOOL   (* Brake release command *)

     (* Status Outputs *)
     G_doPerformanceStatus AT %QX0.4 : BOOL   (* Mode-dependent status indicator *)
     G_doFaultActive       AT %QX0.5 : BOOL   (* Fault indicator: HIGH = fault active *)
     G_doInMotion          AT %QX0.6 : BOOL   (* Axis in motion indicator *)
     G_doHomingComplete    AT %QX0.7 : BOOL   (* Homing sequence complete *)


     (* ========================================================================
        ANALOG INPUTS (1 Available)
        Range: -10V to +10V mapped to INT -32768 to +32767
        Resolution: 20V / 65536 = 0.305 mV/count
        ======================================================================== *)

     G_aiReference         AT %IW0   : INT    (* Mode-dependent reference signal *)
                                             (* Position mode: position reference *)
                                             (* Velocity mode: velocity reference *)
                                             (* Torque mode: torque reference *)


     (* ========================================================================
        ANALOG OUTPUTS (1 Available)
        Range: -10V to +10V mapped to INT -32768 to +32767
        ======================================================================== *)

     G_aoPositionOutput    AT %QW0   : INT    (* Position output signal to master *)
                                             (* Uses two-stage scaling for resolution *)
   END_VAR


   ============================================================================
   2. CONFIGURABLE PARAMETERS
   ============================================================================

   VAR_GLOBAL
     (* ========================================================================
        MECHANICAL CONFIGURATION
        Actuator: Tolomatic RSA64
        Motor: Yaskawa Sigma-7 400W (SGM7J-04A6A6C)
        ======================================================================== *)

     G_cfgGearRatio            : LREAL := 70.0        (* Motor turns per lead screw turn *)
     G_cfgLeadScrewPitch       : LREAL := 5.08        (* mm per lead screw turn (0.2 inch) *)
     G_cfgUsableStroke         : LREAL := 305.0       (* Usable stroke in mm (~12 inches) *)


     (* ========================================================================
        POSITION LIMITS (mm)
        Positive direction: toward cylinder EOT (compression direction)
        ======================================================================== *)

     (* Hard limits - absolute mechanical boundaries *)
     G_cfgPosLimitMax          : LREAL := 320.0       (* Absolute maximum position *)
     G_cfgPosLimitMin          : LREAL := -10.0       (* Absolute minimum position *)

     (* Soft limits - software enforced boundaries *)
     G_cfgPosSoftLimitMax      : LREAL := 305.0       (* Soft limit - extend direction *)
     G_cfgPosSoftLimitMin      : LREAL := 0.0         (* Soft limit - retract direction *)

     (* Critical position landmarks *)
     G_cfgPosCylinderEntry     : LREAL := 20.0        (* Position where piston enters cylinder *)
     G_cfgPosCylinderEOT       : LREAL := 300.0       (* End-of-travel inside cylinder *)
     G_cfgPosExitGuardMargin   : LREAL := 5.0         (* Safety margin for exit prevention *)


     (* ========================================================================
        VELOCITY LIMITS (mm/s)
        ======================================================================== *)

     G_cfgVelLimitMax          : LREAL := 200.0       (* Maximum velocity limit *)
     G_cfgVelLimitNormal       : LREAL := 100.0       (* Normal operating velocity *)
     G_cfgVelHomingFast        : LREAL := 50.0        (* Fast homing approach velocity *)
     G_cfgVelHomingSlow        : LREAL := 5.0         (* Slow homing approach velocity *)
     G_cfgVelGoHome            : LREAL := 50.0        (* Go Home movement velocity *)


     (* ========================================================================
        TORQUE LIMITS (% of rated torque)
        100 = 100% rated, up to 800% peak
        ======================================================================== *)

     G_cfgTorqueLimitMax       : LREAL := 300.0       (* Maximum torque limit *)
     G_cfgTorqueLimitNormal    : LREAL := 100.0       (* Normal operating torque *)
     G_cfgTorqueHomingThreshold : LREAL := 50.0       (* Stall detection threshold for EOT homing *)
     G_cfgTorqueHomingLimit    : LREAL := 60.0        (* Torque limit during slow homing approach *)


     (* ========================================================================
        ANALOG I/O SCALING
        Raw INT range: -32768 to +32767 = -10V to +10V

        Two-stage position mapping provides higher resolution in the
        in-cylinder region where precision matters most.
        ======================================================================== *)

     (* Velocity reference scaling (Mode 011) *)
     G_cfgAnalogVelMin         : LREAL := -100.0      (* Velocity at -10V (mm/s) *)
     G_cfgAnalogVelMax         : LREAL := 100.0       (* Velocity at +10V (mm/s) *)

     (* Torque reference scaling (Mode 100) *)
     G_cfgAnalogTorqueMin      : LREAL := -100.0      (* Torque at -10V (%) *)
     G_cfgAnalogTorqueMax      : LREAL := 100.0       (* Torque at +10V (%) *)

     (* Two-stage position mapping (used for both input and output) *)
     (* Stage 1: In-cylinder region (higher resolution) *)
     G_cfgPosMapStage1PosMin   : LREAL := 0.0         (* Stage 1 position start (mm) *)
     G_cfgPosMapStage1VoltMin  : LREAL := -10.0       (* Stage 1 voltage start (V) *)
     G_cfgPosMapStage1VoltMax  : LREAL := 5.0         (* Stage 1 voltage end (V) *)

     (* Stage transition point *)
     G_cfgPosMapTransitionPos  : LREAL := 200.0       (* Position at stage boundary (mm) *)
     G_cfgPosMapTransitionVolt : LREAL := 5.0         (* Voltage at stage boundary (V) *)

     (* Stage 2: Out-of-cylinder region (lower resolution) *)
     G_cfgPosMapStage2PosMax   : LREAL := 305.0       (* Stage 2 position end (mm) *)
     G_cfgPosMapStage2VoltMax  : LREAL := 10.0        (* Stage 2 voltage end (V) *)


     (* ========================================================================
        TIMING PARAMETERS
        ======================================================================== *)

     G_cfgHandshakeTimeout     : TIME := T#500MS      (* Handshake timeout duration *)
     G_cfgModeTransitionDelay  : TIME := T#100MS      (* Delay between mode transitions *)
     G_cfgBrakeEngageDelay     : TIME := T#200MS      (* Wait after brake engage command *)
     G_cfgBrakeDisengageDelay  : TIME := T#100MS      (* Wait after brake disengage command *)
     G_cfgHomingTimeout        : TIME := T#30S        (* Overall homing sequence timeout *)
     G_cfgStallDetectTime      : TIME := T#200MS      (* Time to confirm stall condition *)
     G_cfgDriveReadyTimeout    : TIME := T#5S         (* Timeout waiting for drive ready *)
     G_cfgFaultIdleTimeout     : TIME := T#30S        (* Timeout in fault before going to idle *)


     (* ========================================================================
        INPUT FILTERING PARAMETERS
        ======================================================================== *)

     (* Digital input debounce times *)
     G_cfgDebounceTimeMode     : TIME := T#50MS       (* Mode bits (DI0-2) debounce *)
     G_cfgDebounceTimeMotion   : TIME := T#20MS       (* Motion enable (DI3) debounce *)
     G_cfgDebounceTimeFault    : TIME := T#20MS       (* Fault reset (DI6) debounce *)
     G_cfgDebounceTimeLimitStd : TIME := T#5MS        (* Limit switches standard debounce *)
     G_cfgDebounceTimeLimitHome: TIME := T#2MS        (* Home limit switch (shorter for precision) *)

     (* Analog input filter parameters *)
     G_cfgAnalogFilterTimeConst: TIME := T#50MS       (* Low-pass filter time constant *)
     G_cfgAnalogMedianSize     : INT := 3             (* Median filter size (3 or 5) *)
     G_cfgCycleTime            : TIME := T#1MS        (* PLC cycle time *)


     (* ========================================================================
        MOTION DETECTION THRESHOLDS
        ======================================================================== *)

     G_cfgInMotionVelThreshold : LREAL := 0.1         (* Velocity threshold for "in motion" (mm/s) *)


     (* ========================================================================
        HOMING PARAMETERS
        ======================================================================== *)

     (* Mode 110 - Home to Limit Switch *)
     G_cfgHomeLimApproachVel   : LREAL := 50.0        (* Approach velocity (mm/s) *)
     G_cfgHomeLimBackoffDist   : LREAL := 5.0         (* Back-off distance after trigger (mm) *)
     G_cfgHomeLimSetPosition   : LREAL := 0.0         (* Position to set at home switch *)

     (* Mode 111 - Home to End-of-Travel *)
     G_cfgHomeEOTFastVel       : LREAL := 50.0        (* Fast approach velocity (mm/s) *)
     G_cfgHomeEOTSlowVel       : LREAL := 5.0         (* Slow torque-limited velocity (mm/s) *)
     G_cfgHomeEOTApproachDist  : LREAL := 20.0        (* Distance for slow approach (mm) *)
     G_cfgHomeEOTTorqueThresh  : LREAL := 50.0        (* Torque threshold for stall (%) *)
     G_cfgHomeEOTSetPosition   : LREAL := 300.0       (* Position to set at EOT *)

     (* Mode 101 - Go Home *)
     G_cfgGoHomePosition       : LREAL := 0.0         (* Target position for Go Home *)
     G_cfgGoHomeVelocity       : LREAL := 50.0        (* Velocity for Go Home move *)
   END_VAR


   ============================================================================
   3. POSITION REFERENCES
   ============================================================================

   Coordinate System:
   - Single zero reference established by homing to limit switch (Mode 110)
   - Positive direction: toward cylinder EOT (compression direction)
   - All positions in millimeters (mm)

   VAR_GLOBAL
     (* ========================================================================
        POSITION LANDMARKS
        These are set during commissioning and/or homing sequences
        ======================================================================== *)

     (* Limit switch positions *)
     G_posLimitRetract         : LREAL := -5.0    (* Position of retract limit switch *)
     G_posLimitHome            : LREAL := 0.0     (* Position of homing limit switch (= home) *)

     (* Cylinder boundaries *)
     G_posCylinderEntry        : LREAL := 20.0    (* Position where piston enters cylinder *)
     G_posCylinderEOT          : LREAL := 300.0   (* End-of-travel inside cylinder *)

     (* Software limits (enforced in all modes) *)
     G_posSoftLimitMin         : LREAL := 0.0     (* Minimum allowed position (exit prevention) *)
     G_posSoftLimitMax         : LREAL := 305.0   (* Maximum allowed position *)


     (* ========================================================================
        HOMING REFERENCE POSITIONS
        These are recorded during homing sequences
        ======================================================================== *)

     (* Mode 110 - Home to Limit Switch *)
     G_posHomeLimitSwitch      : LREAL := 0.0     (* Recorded position at limit switch *)

     (* Mode 111 - Home to End-of-Travel *)
     G_posHomeEOT              : LREAL := 0.0     (* Recorded position at EOT stall *)

     (* Combined home reference *)
     G_posHomeOffset           : LREAL := 0.0     (* Offset between limit switch and EOT *)


     (* ========================================================================
        DYNAMIC POSITION TRACKING
        ======================================================================== *)

     (* Position setpoint for Go Home (Mode 101) *)
     G_posGoHomeTarget         : LREAL := 0.0     (* Target position for Go Home move *)

     (* Position at mode entry (for reference) *)
     G_posModeEntryPosition    : LREAL := 0.0     (* Position when entering current mode *)

     (* Exit guard boundary *)
     G_posExitGuardBoundary    : LREAL := 0.0     (* Dynamic boundary for exit prevention *)


     (* ========================================================================
        POSITION VALIDITY FLAGS
        ======================================================================== *)

     G_posAbsoluteValid        : BOOL := FALSE    (* Absolute position from encoder is valid *)
     G_posHomeLimitValid       : BOOL := FALSE    (* Home limit switch position is valid *)
     G_posHomeEOTValid         : BOOL := FALSE    (* EOT position is valid *)
   END_VAR


   ============================================================================
   4. RUNTIME STATE VARIABLES
   ============================================================================

   VAR_GLOBAL
     (* ========================================================================
        STATE MACHINE
        ======================================================================== *)

     G_sysCurrentState         : E_SystemState := ST_INIT     (* Current state *)
     G_sysPreviousState        : E_SystemState := ST_INIT     (* Previous state *)
     G_sysRequestedMode        : E_OperatingMode := MODE_IDLE (* Mode from master *)
     G_sysConfirmedMode        : E_OperatingMode := MODE_IDLE (* Mode confirmed to master *)
     G_sysActiveFault          : E_FaultCode := FAULT_NONE    (* Active fault code *)


     (* ========================================================================
        HOMING FLAGS
        ======================================================================== *)

     (* These flags track homing requirements *)
     G_flagAbsHomeRequired     : BOOL := FALSE    (* Set if encoder invalid, cleared by Mode 110 *)
     G_flagEOTHomeRequired     : BOOL := TRUE     (* Set on power-up, cleared by Mode 111 *)
     G_flagHomingInProgress    : BOOL := FALSE    (* TRUE while homing sequence active *)
     G_flagHomingComplete      : BOOL := FALSE    (* TRUE after successful homing *)


     (* ========================================================================
        DUAL COORDINATE SYSTEM

        Slave coordinates: Encoder position with zero at limit switch
        - Based on absolute encoder, zeroed during limit switch homing
        - Used internally for motion control

        Master coordinates: Master's view based on cylinder geometry
        - Communicated via analog signals from the master controller
        - EOT expected at G_cfgHomeEOTSetPosition
        ======================================================================== *)

     G_posActualEOT            : LREAL := 0.0     (* Encoder position at EOT stall (mm) *)
     G_posEOTOffset            : LREAL := 0.0     (* Offset: master = slave + offset (mm) *)
     (*
        After EOT homing:
        G_posEOTOffset = G_cfgHomeEOTSetPosition - G_posActualEOT

        Position transforms (slave is responsible for translation):
        - Master -> Slave: G_sysCommandedPosition = MasterPosition - G_posEOTOffset
        - Slave -> Master: MasterPosition = G_sysActualPosition + G_posEOTOffset
     *)


     (* ========================================================================
        MOTION FEEDBACK
        Current state of the servo motor/actuator
        ======================================================================== *)

     G_sysActualPosition       : LREAL := 0.0     (* Current position (mm) *)
     G_sysActualVelocity       : LREAL := 0.0     (* Current velocity (mm/s) *)
     G_sysActualTorque         : LREAL := 0.0     (* Current torque (% of rated) *)

     G_sysCommandedPosition    : LREAL := 0.0     (* Commanded position (mm) *)
     G_sysCommandedVelocity    : LREAL := 0.0     (* Commanded velocity (mm/s) *)
     G_sysCommandedTorque      : LREAL := 0.0     (* Commanded torque (%) *)


     (* ========================================================================
        STATUS FLAGS
        ======================================================================== *)

     (* Drive status *)
     G_sysDriveReady           : BOOL := FALSE    (* Drive is ready for motion *)
     G_sysDriveEnabled         : BOOL := FALSE    (* Drive power is enabled *)
     G_sysDriveFault           : BOOL := FALSE    (* Drive has reported a fault *)

     (* Motion status *)
     G_sysMotionActive         : BOOL := FALSE    (* Motion command is active *)
     G_sysInPosition           : BOOL := FALSE    (* At commanded position (position mode) *)
     G_sysAtVelocity           : BOOL := FALSE    (* At commanded velocity (velocity mode) *)
     G_sysHalted               : BOOL := FALSE    (* Halt is active (ST_HOLD_POSITION) *)

     (* Limit status *)
     G_sysAtPosLimitHigh       : BOOL := FALSE    (* At or beyond high position limit *)
     G_sysAtPosLimitLow        : BOOL := FALSE    (* At or beyond low position limit *)
     G_sysLimitRetractActive   : BOOL := FALSE    (* Retract limit switch triggered *)
     G_sysLimitHomeActive      : BOOL := FALSE    (* Home limit switch triggered *)

     (* Safety status *)
     G_sysExitGuardActive      : BOOL := FALSE    (* Piston exit prevention active *)
     G_sysInCylinderZone       : BOOL := FALSE    (* Piston is inside cylinder *)

     (* Brake status *)
     G_sysBrakeEngaged         : BOOL := TRUE     (* Brake is engaged (spring-applied) *)


     (* ========================================================================
        DIAGNOSTICS
        ======================================================================== *)

     G_sysCycleCount           : UDINT := 0       (* Main program cycle counter *)
     G_sysLastFaultCode        : E_FaultCode := FAULT_NONE    (* Last fault that occurred *)
     G_sysLastFaultState       : E_SystemState := ST_INIT     (* State when last fault occurred *)
     G_sysFaultCount           : UDINT := 0       (* Total number of faults since power-up *)

     (* Timing diagnostics *)
     G_sysStateEntryTime       : TIME := T#0MS    (* Time when current state was entered *)
     G_sysTimeInState          : TIME := T#0MS    (* Duration in current state *)

     (* Encoder status *)
     G_sysEncoderValid         : BOOL := FALSE    (* Absolute encoder position is valid *)
     G_sysEncoderBatteryOK     : BOOL := FALSE    (* Encoder battery status OK *)
   END_VAR


   ============================================================================
   5. COMMAND / STATUS INTERFACE (ST <-> LD POU)
   ============================================================================

   These structured globals connect the ST state machine (PRG_Main) to the
   Ladder Logic POU that owns G_sysAxis and all built-in motion FBs.

   Commands flow ST -> LD (written by PRG_Main, read by LD POU).
   Status flows LD -> ST (written by LD POU, read by PRG_Main and custom FBs).

   VAR_GLOBAL
     (* ========================================================================
        COMMAND STRUCTS (ST -> LD)
        ======================================================================== *)

     G_cmdPower            : ST_CmdPower
     G_cmdStop             : ST_CmdStop
     G_cmdReset            : ST_CmdReset
     G_cmdMoveAbsolute     : ST_CmdMoveAbsolute
     G_cmdMoveVelocity     : ST_CmdMoveVelocity
     G_cmdDirectControl    : ST_CmdDirectControl
     G_cmdEncMngr     : ST_CmdEncMngr


     (* ========================================================================
        STATUS STRUCTS (LD -> ST)
        ======================================================================== *)

     G_staPower            : ST_StaPower
     G_staStop             : ST_StaStop
     G_staReset            : ST_StaReset
     G_staMoveAbsolute     : ST_StaMoveAbsolute
     G_staMoveVelocity     : ST_StaMoveVelocity
     G_staDirectControl    : ST_StaDirectControl
     G_staEncMngr     : ST_StaEncMngr
   END_VAR

*)
