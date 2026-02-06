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
     diModeBit0          AT %IX0.0 : BOOL   (* Mode bit 0 - LSB *)
     diModeBit1          AT %IX0.1 : BOOL   (* Mode bit 1 *)
     diModeBit2          AT %IX0.2 : BOOL   (* Mode bit 2 - MSB *)

     (* Motion Control *)
     diMotionEnable      AT %IX0.3 : BOOL   (* Motion enable from master *)

     (* Limit Switches - Tolomatic #8100-9092, Solid State, NC, PNP *)
     (* TRUE = switch NOT triggered (normal), FALSE = triggered or wire broken *)
     diLimitRetract      AT %IX0.4 : BOOL   (* Retracted position limit switch *)
     diLimitHome         AT %IX0.5 : BOOL   (* Homing reference limit switch *)

     (* Fault Control *)
     diFaultReset        AT %IX0.6 : BOOL   (* Fault reset command from master *)

     (* Reserved *)
     diReserved          AT %IX0.7 : BOOL   (* Reserved for future use *)


     (* ========================================================================
        DIGITAL OUTPUTS (8 Available)
        ======================================================================== *)

     (* Mode Confirmation to Master (3-bit) / Fault Code when doFaultActive=TRUE *)
     doModeConfBit0      AT %QX0.0 : BOOL   (* Mode confirm bit 0 / Fault code bit 0 *)
     doModeConfBit1      AT %QX0.1 : BOOL   (* Mode confirm bit 1 / Fault code bit 1 *)
     doModeConfBit2      AT %QX0.2 : BOOL   (* Mode confirm bit 2 / Fault code bit 2 *)

     (* Brake Control *)
     (* Spring-engaged, electrically released: HIGH = brake disengaged (motion allowed) *)
     doBrakeDisengage    AT %QX0.3 : BOOL   (* Brake release command *)

     (* Status Outputs *)
     doPerformanceStatus AT %QX0.4 : BOOL   (* Mode-dependent status indicator *)
     doFaultActive       AT %QX0.5 : BOOL   (* Fault indicator: HIGH = fault active *)
     doInMotion          AT %QX0.6 : BOOL   (* Axis in motion indicator *)
     doHomingComplete    AT %QX0.7 : BOOL   (* Homing sequence complete *)


     (* ========================================================================
        ANALOG INPUTS (1 Available)
        Range: -10V to +10V mapped to INT -32768 to +32767
        Resolution: 20V / 65536 = 0.305 mV/count
        ======================================================================== *)

     aiReference         AT %IW0   : INT    (* Mode-dependent reference signal *)
                                             (* Position mode: position reference *)
                                             (* Velocity mode: velocity reference *)
                                             (* Torque mode: torque reference *)


     (* ========================================================================
        ANALOG OUTPUTS (1 Available)
        Range: -10V to +10V mapped to INT -32768 to +32767
        ======================================================================== *)

     aoPositionOutput    AT %QW0   : INT    (* Position output signal to master *)
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

     cfgGearRatio            : LREAL := 70.0        (* Motor turns per lead screw turn *)
     cfgLeadScrewPitch       : LREAL := 5.08        (* mm per lead screw turn (0.2 inch) *)
     cfgUsableStroke         : LREAL := 305.0       (* Usable stroke in mm (~12 inches) *)


     (* ========================================================================
        POSITION LIMITS (mm)
        Positive direction: toward cylinder EOT (compression direction)
        ======================================================================== *)

     (* Hard limits - absolute mechanical boundaries *)
     cfgPosLimitMax          : LREAL := 320.0       (* Absolute maximum position *)
     cfgPosLimitMin          : LREAL := -10.0       (* Absolute minimum position *)

     (* Soft limits - software enforced boundaries *)
     cfgPosSoftLimitMax      : LREAL := 305.0       (* Soft limit - extend direction *)
     cfgPosSoftLimitMin      : LREAL := 0.0         (* Soft limit - retract direction *)

     (* Critical position landmarks *)
     cfgPosCylinderEntry     : LREAL := 20.0        (* Position where piston enters cylinder *)
     cfgPosCylinderEOT       : LREAL := 300.0       (* End-of-travel inside cylinder *)
     cfgPosExitGuardMargin   : LREAL := 5.0         (* Safety margin for exit prevention *)


     (* ========================================================================
        VELOCITY LIMITS (mm/s)
        ======================================================================== *)

     cfgVelLimitMax          : LREAL := 200.0       (* Maximum velocity limit *)
     cfgVelLimitNormal       : LREAL := 100.0       (* Normal operating velocity *)
     cfgVelHomingFast        : LREAL := 50.0        (* Fast homing approach velocity *)
     cfgVelHomingSlow        : LREAL := 5.0         (* Slow homing approach velocity *)
     cfgVelGoHome            : LREAL := 50.0        (* Go Home movement velocity *)


     (* ========================================================================
        TORQUE LIMITS (% of rated torque)
        100 = 100% rated, up to 800% peak
        ======================================================================== *)

     cfgTorqueLimitMax       : LREAL := 300.0       (* Maximum torque limit *)
     cfgTorqueLimitNormal    : LREAL := 100.0       (* Normal operating torque *)
     cfgTorqueHomingThreshold : LREAL := 50.0       (* Stall detection threshold for EOT homing *)
     cfgTorqueHomingLimit    : LREAL := 60.0        (* Torque limit during slow homing approach *)


     (* ========================================================================
        ANALOG I/O SCALING
        Raw INT range: -32768 to +32767 = -10V to +10V

        Two-stage position mapping provides higher resolution in the
        in-cylinder region where precision matters most.
        ======================================================================== *)

     (* Velocity reference scaling (Mode 011) *)
     cfgAnalogVelMin         : LREAL := -100.0      (* Velocity at -10V (mm/s) *)
     cfgAnalogVelMax         : LREAL := 100.0       (* Velocity at +10V (mm/s) *)

     (* Torque reference scaling (Mode 100) *)
     cfgAnalogTorqueMin      : LREAL := -100.0      (* Torque at -10V (%) *)
     cfgAnalogTorqueMax      : LREAL := 100.0       (* Torque at +10V (%) *)

     (* Two-stage position mapping (used for both input and output) *)
     (* Stage 1: In-cylinder region (higher resolution) *)
     cfgPosMapStage1PosMin   : LREAL := 0.0         (* Stage 1 position start (mm) *)
     cfgPosMapStage1VoltMin  : LREAL := -10.0       (* Stage 1 voltage start (V) *)
     cfgPosMapStage1VoltMax  : LREAL := 5.0         (* Stage 1 voltage end (V) *)

     (* Stage transition point *)
     cfgPosMapTransitionPos  : LREAL := 200.0       (* Position at stage boundary (mm) *)
     cfgPosMapTransitionVolt : LREAL := 5.0         (* Voltage at stage boundary (V) *)

     (* Stage 2: Out-of-cylinder region (lower resolution) *)
     cfgPosMapStage2PosMax   : LREAL := 305.0       (* Stage 2 position end (mm) *)
     cfgPosMapStage2VoltMax  : LREAL := 10.0        (* Stage 2 voltage end (V) *)


     (* ========================================================================
        TIMING PARAMETERS
        ======================================================================== *)

     cfgHandshakeTimeout     : TIME := T#500MS      (* Handshake timeout duration *)
     cfgModeTransitionDelay  : TIME := T#100MS      (* Delay between mode transitions *)
     cfgBrakeEngageDelay     : TIME := T#200MS      (* Wait after brake engage command *)
     cfgBrakeDisengageDelay  : TIME := T#100MS      (* Wait after brake disengage command *)
     cfgHomingTimeout        : TIME := T#30S        (* Overall homing sequence timeout *)
     cfgStallDetectTime      : TIME := T#200MS      (* Time to confirm stall condition *)
     cfgDriveReadyTimeout    : TIME := T#5S         (* Timeout waiting for drive ready *)
     cfgFaultIdleTimeout     : TIME := T#30S        (* Timeout in fault before going to idle *)


     (* ========================================================================
        INPUT FILTERING PARAMETERS
        ======================================================================== *)

     (* Digital input debounce times *)
     cfgDebounceTimeMode     : TIME := T#50MS       (* Mode bits (DI0-2) debounce *)
     cfgDebounceTimeMotion   : TIME := T#20MS       (* Motion enable (DI3) debounce *)
     cfgDebounceTimeFault    : TIME := T#20MS       (* Fault reset (DI6) debounce *)
     cfgDebounceTimeLimitStd : TIME := T#5MS        (* Limit switches standard debounce *)
     cfgDebounceTimeLimitHome: TIME := T#2MS        (* Home limit switch (shorter for precision) *)

     (* Analog input filter parameters *)
     cfgAnalogFilterTimeConst: TIME := T#50MS       (* Low-pass filter time constant *)
     cfgAnalogMedianSize     : INT := 3             (* Median filter size (3 or 5) *)
     cfgCycleTime            : TIME := T#1MS        (* PLC cycle time *)


     (* ========================================================================
        MOTION DETECTION THRESHOLDS
        ======================================================================== *)

     cfgInMotionVelThreshold : LREAL := 0.1         (* Velocity threshold for "in motion" (mm/s) *)


     (* ========================================================================
        HOMING PARAMETERS
        ======================================================================== *)

     (* Mode 110 - Home to Limit Switch *)
     cfgHomeLimApproachVel   : LREAL := 50.0        (* Approach velocity (mm/s) *)
     cfgHomeLimBackoffDist   : LREAL := 5.0         (* Back-off distance after trigger (mm) *)
     cfgHomeLimSetPosition   : LREAL := 0.0         (* Position to set at home switch *)

     (* Mode 111 - Home to End-of-Travel *)
     cfgHomeEOTFastVel       : LREAL := 50.0        (* Fast approach velocity (mm/s) *)
     cfgHomeEOTSlowVel       : LREAL := 5.0         (* Slow torque-limited velocity (mm/s) *)
     cfgHomeEOTApproachDist  : LREAL := 20.0        (* Distance for slow approach (mm) *)
     cfgHomeEOTTorqueThresh  : LREAL := 50.0        (* Torque threshold for stall (%) *)
     cfgHomeEOTSetPosition   : LREAL := 300.0       (* Position to set at EOT *)

     (* Mode 101 - Go Home *)
     cfgGoHomePosition       : LREAL := 0.0         (* Target position for Go Home *)
     cfgGoHomeVelocity       : LREAL := 50.0        (* Velocity for Go Home move *)
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
     posLimitRetract         : LREAL := -5.0    (* Position of retract limit switch *)
     posLimitHome            : LREAL := 0.0     (* Position of homing limit switch (= home) *)

     (* Cylinder boundaries *)
     posCylinderEntry        : LREAL := 20.0    (* Position where piston enters cylinder *)
     posCylinderEOT          : LREAL := 300.0   (* End-of-travel inside cylinder *)

     (* Software limits (enforced in all modes) *)
     posSoftLimitMin         : LREAL := 0.0     (* Minimum allowed position (exit prevention) *)
     posSoftLimitMax         : LREAL := 305.0   (* Maximum allowed position *)


     (* ========================================================================
        HOMING REFERENCE POSITIONS
        These are recorded during homing sequences
        ======================================================================== *)

     (* Mode 110 - Home to Limit Switch *)
     posHomeLimitSwitch      : LREAL := 0.0     (* Recorded position at limit switch *)

     (* Mode 111 - Home to End-of-Travel *)
     posHomeEOT              : LREAL := 0.0     (* Recorded position at EOT stall *)

     (* Combined home reference *)
     posHomeOffset           : LREAL := 0.0     (* Offset between limit switch and EOT *)


     (* ========================================================================
        DYNAMIC POSITION TRACKING
        ======================================================================== *)

     (* Position setpoint for Go Home (Mode 101) *)
     posGoHomeTarget         : LREAL := 0.0     (* Target position for Go Home move *)

     (* Position at mode entry (for reference) *)
     posModeEntryPosition    : LREAL := 0.0     (* Position when entering current mode *)

     (* Exit guard boundary *)
     posExitGuardBoundary    : LREAL := 0.0     (* Dynamic boundary for exit prevention *)


     (* ========================================================================
        POSITION VALIDITY FLAGS
        ======================================================================== *)

     posAbsoluteValid        : BOOL := FALSE    (* Absolute position from encoder is valid *)
     posHomeLimitValid       : BOOL := FALSE    (* Home limit switch position is valid *)
     posHomeEOTValid         : BOOL := FALSE    (* EOT position is valid *)
   END_VAR


   ============================================================================
   4. RUNTIME STATE VARIABLES
   ============================================================================

   VAR_GLOBAL
     (* ========================================================================
        STATE MACHINE
        ======================================================================== *)

     sysCurrentState         : E_SystemState := ST_INIT     (* Current state *)
     sysPreviousState        : E_SystemState := ST_INIT     (* Previous state *)
     sysRequestedMode        : E_OperatingMode := MODE_IDLE (* Mode from master *)
     sysConfirmedMode        : E_OperatingMode := MODE_IDLE (* Mode confirmed to master *)
     sysActiveFault          : E_FaultCode := FAULT_NONE    (* Active fault code *)


     (* ========================================================================
        HOMING FLAGS
        ======================================================================== *)

     (* These flags track homing requirements *)
     flagAbsHomeRequired     : BOOL := FALSE    (* Set if encoder invalid, cleared by Mode 110 *)
     flagEOTHomeRequired     : BOOL := TRUE     (* Set on power-up, cleared by Mode 111 *)
     flagHomingInProgress    : BOOL := FALSE    (* TRUE while homing sequence active *)
     flagHomingComplete      : BOOL := FALSE    (* TRUE after successful homing *)


     (* ========================================================================
        DUAL COORDINATE SYSTEM

        Slave coordinates: Encoder position with zero at limit switch
        - Based on absolute encoder, zeroed during limit switch homing
        - Used internally for motion control

        Master coordinates: Master's view based on cylinder geometry
        - Communicated via analog signals from the master controller
        - EOT expected at cfgHomeEOTSetPosition
        ======================================================================== *)

     posActualEOT            : LREAL := 0.0     (* Encoder position at EOT stall (mm) *)
     posEOTOffset            : LREAL := 0.0     (* Offset: master = slave + offset (mm) *)
     (*
        After EOT homing:
        posEOTOffset = cfgHomeEOTSetPosition - posActualEOT

        Position transforms (slave is responsible for translation):
        - Master -> Slave: sysCommandedPosition = MasterPosition - posEOTOffset
        - Slave -> Master: MasterPosition = sysActualPosition + posEOTOffset
     *)


     (* ========================================================================
        AXIS REFERENCE
        PLCopen axis structure for motion control function blocks
        ======================================================================== *)

     sysAxis                 : AXIS_REF         (* PLCopen axis reference *)


     (* ========================================================================
        MOTION FEEDBACK
        Current state of the servo motor/actuator
        ======================================================================== *)

     sysActualPosition       : LREAL := 0.0     (* Current position (mm) *)
     sysActualVelocity       : LREAL := 0.0     (* Current velocity (mm/s) *)
     sysActualTorque         : LREAL := 0.0     (* Current torque (% of rated) *)

     sysCommandedPosition    : LREAL := 0.0     (* Commanded position (mm) *)
     sysCommandedVelocity    : LREAL := 0.0     (* Commanded velocity (mm/s) *)
     sysCommandedTorque      : LREAL := 0.0     (* Commanded torque (%) *)


     (* ========================================================================
        STATUS FLAGS
        ======================================================================== *)

     (* Drive status *)
     sysDriveReady           : BOOL := FALSE    (* Drive is ready for motion *)
     sysDriveEnabled         : BOOL := FALSE    (* Drive power is enabled *)
     sysDriveFault           : BOOL := FALSE    (* Drive has reported a fault *)

     (* Motion status *)
     sysMotionActive         : BOOL := FALSE    (* Motion command is active *)
     sysInPosition           : BOOL := FALSE    (* At commanded position (position mode) *)
     sysAtVelocity           : BOOL := FALSE    (* At commanded velocity (velocity mode) *)
     sysHalted               : BOOL := FALSE    (* Halt is active (ST_HOLD_POSITION) *)

     (* Limit status *)
     sysAtPosLimitHigh       : BOOL := FALSE    (* At or beyond high position limit *)
     sysAtPosLimitLow        : BOOL := FALSE    (* At or beyond low position limit *)
     sysLimitRetractActive   : BOOL := FALSE    (* Retract limit switch triggered *)
     sysLimitHomeActive      : BOOL := FALSE    (* Home limit switch triggered *)

     (* Safety status *)
     sysExitGuardActive      : BOOL := FALSE    (* Piston exit prevention active *)
     sysInCylinderZone       : BOOL := FALSE    (* Piston is inside cylinder *)

     (* Brake status *)
     sysBrakeEngaged         : BOOL := TRUE     (* Brake is engaged (spring-applied) *)


     (* ========================================================================
        HANDSHAKE STATUS
        ======================================================================== *)

     sysHandshakeValid       : BOOL := FALSE    (* Current handshake is valid *)
     sysHandshakeComplete    : BOOL := FALSE    (* Handshake sequence complete *)
     sysHandshakeTimeout     : BOOL := FALSE    (* Handshake timed out *)
     sysFaultResetPending    : BOOL := FALSE    (* Fault reset handshake in progress *)


     (* ========================================================================
        DIAGNOSTICS
        ======================================================================== *)

     sysCycleCount           : UDINT := 0       (* Main program cycle counter *)
     sysLastFaultCode        : E_FaultCode := FAULT_NONE    (* Last fault that occurred *)
     sysLastFaultState       : E_SystemState := ST_INIT     (* State when last fault occurred *)
     sysFaultCount           : UDINT := 0       (* Total number of faults since power-up *)

     (* Timing diagnostics *)
     sysStateEntryTime       : TIME := T#0MS    (* Time when current state was entered *)
     sysTimeInState          : TIME := T#0MS    (* Duration in current state *)

     (* Encoder status *)
     sysEncoderValid         : BOOL := FALSE    (* Absolute encoder position is valid *)
     sysEncoderBatteryOK     : BOOL := FALSE    (* Encoder battery status OK *)
   END_VAR

*)
