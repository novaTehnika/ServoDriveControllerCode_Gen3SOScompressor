(* ============================================================================
   GVL_Config - Global Variable List for Configurable Parameters

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This GVL contains all configurable parameters for the servo controller
   including mechanical configuration, limits, timing, and homing parameters.

   Units:
   - Position: millimeters (mm)
   - Velocity: mm/s
   - Torque: % of rated (100 = 100%, up to 800% peak)
   - Time: TIME type (T#xxxMS, T#xxxS)
   ============================================================================ *)

VAR_GLOBAL
    (* ========================================================================
       MECHANICAL CONFIGURATION
       Actuator: Tolomatic RSA64
       Motor: Yaskawa Sigma-7 400W (SGM7J-04A6A6C)
       ======================================================================== *)

    cfgGearRatio            : LREAL := 70.0;        (* Motor turns per lead screw turn *)
    cfgLeadScrewPitch       : LREAL := 5.08;        (* mm per lead screw turn (0.2 inch) *)
    cfgUsableStroke         : LREAL := 305.0;       (* Usable stroke in mm (~12 inches) *)


    (* ========================================================================
       POSITION LIMITS (mm)
       Positive direction: toward cylinder EOT (compression direction)
       ======================================================================== *)

    (* Hard limits - absolute mechanical boundaries *)
    cfgPosLimitMax          : LREAL := 320.0;       (* Absolute maximum position *)
    cfgPosLimitMin          : LREAL := -10.0;       (* Absolute minimum position *)

    (* Soft limits - software enforced boundaries *)
    cfgPosSoftLimitMax      : LREAL := 305.0;       (* Soft limit - extend direction *)
    cfgPosSoftLimitMin      : LREAL := 0.0;         (* Soft limit - retract direction *)

    (* Critical position landmarks *)
    cfgPosCylinderEntry     : LREAL := 20.0;        (* Position where piston enters cylinder *)
    cfgPosCylinderEOT       : LREAL := 300.0;       (* End-of-travel inside cylinder *)
    cfgPosExitGuardMargin   : LREAL := 5.0;         (* Safety margin for exit prevention *)


    (* ========================================================================
       VELOCITY LIMITS (mm/s)
       ======================================================================== *)

    cfgVelLimitMax          : LREAL := 200.0;       (* Maximum velocity limit *)
    cfgVelLimitNormal       : LREAL := 100.0;       (* Normal operating velocity *)
    cfgVelHomingFast        : LREAL := 50.0;        (* Fast homing approach velocity *)
    cfgVelHomingSlow        : LREAL := 5.0;         (* Slow homing approach velocity *)
    cfgVelGoHome            : LREAL := 50.0;        (* Go Home movement velocity *)


    (* ========================================================================
       TORQUE LIMITS (% of rated torque)
       100 = 100% rated, up to 800% peak
       ======================================================================== *)

    cfgTorqueLimitMax       : LREAL := 300.0;       (* Maximum torque limit *)
    cfgTorqueLimitNormal    : LREAL := 100.0;       (* Normal operating torque *)
    cfgTorqueHomingThreshold : LREAL := 50.0;       (* Stall detection threshold for EOT homing *)
    cfgTorqueHomingLimit    : LREAL := 60.0;        (* Torque limit during slow homing approach *)


    (* ========================================================================
       ANALOG I/O SCALING
       Raw INT range: -32768 to +32767 = -10V to +10V

       Two-stage position mapping provides higher resolution in the
       in-cylinder region where precision matters most.
       ======================================================================== *)

    (* Velocity reference scaling (Mode 011) *)
    cfgAnalogVelMin         : LREAL := -100.0;      (* Velocity at -10V (mm/s) *)
    cfgAnalogVelMax         : LREAL := 100.0;       (* Velocity at +10V (mm/s) *)

    (* Torque reference scaling (Mode 100) *)
    cfgAnalogTorqueMin      : LREAL := -100.0;      (* Torque at -10V (%) *)
    cfgAnalogTorqueMax      : LREAL := 100.0;       (* Torque at +10V (%) *)

    (* Two-stage position mapping (used for both input and output) *)
    (* Stage 1: In-cylinder region (higher resolution) *)
    cfgPosMapStage1PosMin   : LREAL := 0.0;         (* Stage 1 position start (mm) *)
    cfgPosMapStage1VoltMin  : LREAL := -10.0;       (* Stage 1 voltage start (V) *)
    cfgPosMapStage1VoltMax  : LREAL := 5.0;         (* Stage 1 voltage end (V) *)

    (* Stage transition point *)
    cfgPosMapTransitionPos  : LREAL := 200.0;       (* Position at stage boundary (mm) *)
    cfgPosMapTransitionVolt : LREAL := 5.0;         (* Voltage at stage boundary (V) *)

    (* Stage 2: Out-of-cylinder region (lower resolution) *)
    cfgPosMapStage2PosMax   : LREAL := 305.0;       (* Stage 2 position end (mm) *)
    cfgPosMapStage2VoltMax  : LREAL := 10.0;        (* Stage 2 voltage end (V) *)


    (* ========================================================================
       TIMING PARAMETERS
       ======================================================================== *)

    cfgHandshakeTimeout     : TIME := T#500MS;      (* Handshake timeout duration *)
    cfgModeTransitionDelay  : TIME := T#100MS;      (* Delay between mode transitions *)
    cfgBrakeEngageDelay     : TIME := T#200MS;      (* Wait after brake engage command *)
    cfgBrakeDisengageDelay  : TIME := T#100MS;      (* Wait after brake disengage command *)
    cfgHomingTimeout        : TIME := T#30S;        (* Overall homing sequence timeout *)
    cfgStallDetectTime      : TIME := T#200MS;      (* Time to confirm stall condition *)
    cfgDriveReadyTimeout    : TIME := T#5S;         (* Timeout waiting for drive ready *)
    cfgFaultIdleTimeout     : TIME := T#30S;        (* Timeout in fault before going to idle *)


    (* ========================================================================
       INPUT FILTERING PARAMETERS
       ======================================================================== *)

    (* Digital input debounce times *)
    cfgDebounceTimeMode     : TIME := T#50MS;       (* Mode bits (DI0-2) debounce *)
    cfgDebounceTimeMotion   : TIME := T#20MS;       (* Motion enable (DI3) debounce *)
    cfgDebounceTimeFault    : TIME := T#20MS;       (* Fault reset (DI6) debounce *)
    cfgDebounceTimeLimitStd : TIME := T#5MS;        (* Limit switches standard debounce *)
    cfgDebounceTimeLimitHome: TIME := T#2MS;        (* Home limit switch (shorter for precision) *)

    (* Analog input filter parameters *)
    cfgAnalogFilterTimeConst: TIME := T#50MS;       (* Low-pass filter time constant *)
    cfgAnalogMedianSize     : INT := 3;             (* Median filter size (3 or 5) *)
    cfgCycleTime            : TIME := T#1MS;        (* PLC cycle time *)


    (* ========================================================================
       MOTION DETECTION THRESHOLDS
       ======================================================================== *)

    cfgInMotionVelThreshold : LREAL := 0.1;         (* Velocity threshold for "in motion" (mm/s) *)


    (* ========================================================================
       HOMING PARAMETERS
       ======================================================================== *)

    (* Mode 110 - Home to Limit Switch *)
    cfgHomeLimApproachVel   : LREAL := 50.0;        (* Approach velocity (mm/s) *)
    cfgHomeLimBackoffDist   : LREAL := 5.0;         (* Back-off distance after trigger (mm) *)
    cfgHomeLimSetPosition   : LREAL := 0.0;         (* Position to set at home switch *)

    (* Mode 111 - Home to End-of-Travel *)
    cfgHomeEOTFastVel       : LREAL := 50.0;        (* Fast approach velocity (mm/s) *)
    cfgHomeEOTSlowVel       : LREAL := 5.0;         (* Slow torque-limited velocity (mm/s) *)
    cfgHomeEOTApproachDist  : LREAL := 20.0;        (* Distance for slow approach (mm) *)
    cfgHomeEOTTorqueThresh  : LREAL := 50.0;        (* Torque threshold for stall (%) *)
    cfgHomeEOTSetPosition   : LREAL := 300.0;       (* Position to set at EOT *)

    (* Mode 101 - Go Home *)
    cfgGoHomePosition       : LREAL := 0.0;         (* Target position for Go Home *)
    cfgGoHomeVelocity       : LREAL := 50.0;        (* Velocity for Go Home move *)

END_VAR
