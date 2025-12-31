(* ============================================================================
   GVL_System - Global Variable List for Runtime State Variables

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This GVL contains all runtime state variables including state machine state,
   axis feedback, status flags, and diagnostic information.
   ============================================================================ *)

VAR_GLOBAL
    (* ========================================================================
       STATE MACHINE
       ======================================================================== *)

    sysCurrentState         : E_SystemState := ST_INIT;     (* Current state *)
    sysPreviousState        : E_SystemState := ST_INIT;     (* Previous state *)
    sysRequestedMode        : E_OperatingMode := MODE_IDLE; (* Mode from master *)
    sysConfirmedMode        : E_OperatingMode := MODE_IDLE; (* Mode confirmed to master *)
    sysActiveFault          : E_FaultCode := FAULT_NONE;    (* Active fault code *)


    (* ========================================================================
       HOMING FLAGS
       ======================================================================== *)

    (* These flags track homing requirements *)
    flagAbsHomeRequired     : BOOL := FALSE;    (* Set if encoder invalid, cleared by Mode 110 *)
    flagEOTHomeRequired     : BOOL := TRUE;     (* Set on power-up, cleared by Mode 111 *)
    flagHomingInProgress    : BOOL := FALSE;    (* TRUE while homing sequence active *)
    flagHomingComplete      : BOOL := FALSE;    (* TRUE after successful homing *)


    (* ========================================================================
       DUAL COORDINATE SYSTEM

       Slave coordinates: Encoder position with zero at limit switch
       - Based on absolute encoder, zeroed during limit switch homing
       - Used internally for motion control

       Master coordinates: Master's view based on cylinder geometry
       - Communicated via analog signals from the master controller
       - EOT expected at cfgHomeEOTSetPosition
       ======================================================================== *)

    posActualEOT            : LREAL := 0.0;     (* Encoder position at EOT stall (mm) *)
    posEOTOffset            : LREAL := 0.0;     (* Offset: master = slave + offset (mm) *)
    (*
       After EOT homing:
       posEOTOffset = cfgHomeEOTSetPosition - posActualEOT

       Position transforms (slave is responsible for translation):
       - Master → Slave: sysCommandedPosition = MasterPosition - posEOTOffset
       - Slave → Master: MasterPosition = sysActualPosition + posEOTOffset
    *)


    (* ========================================================================
       AXIS REFERENCE
       PLCopen axis structure for motion control function blocks
       ======================================================================== *)

    sysAxis                 : AXIS_REF;         (* PLCopen axis reference *)


    (* ========================================================================
       MOTION FEEDBACK
       Current state of the servo motor/actuator
       ======================================================================== *)

    sysActualPosition       : LREAL := 0.0;     (* Current position (mm) *)
    sysActualVelocity       : LREAL := 0.0;     (* Current velocity (mm/s) *)
    sysActualTorque         : LREAL := 0.0;     (* Current torque (% of rated) *)

    sysCommandedPosition    : LREAL := 0.0;     (* Commanded position (mm) *)
    sysCommandedVelocity    : LREAL := 0.0;     (* Commanded velocity (mm/s) *)
    sysCommandedTorque      : LREAL := 0.0;     (* Commanded torque (%) *)


    (* ========================================================================
       STATUS FLAGS
       ======================================================================== *)

    (* Drive status *)
    sysDriveReady           : BOOL := FALSE;    (* Drive is ready for motion *)
    sysDriveEnabled         : BOOL := FALSE;    (* Drive power is enabled *)
    sysDriveFault           : BOOL := FALSE;    (* Drive has reported a fault *)

    (* Motion status *)
    sysMotionActive         : BOOL := FALSE;    (* Motion command is active *)
    sysInPosition           : BOOL := FALSE;    (* At commanded position (position mode) *)
    sysAtVelocity           : BOOL := FALSE;    (* At commanded velocity (velocity mode) *)
    sysHalted               : BOOL := FALSE;    (* Halt is active (ST_HOLD_POSITION) *)

    (* Limit status *)
    sysAtPosLimitHigh       : BOOL := FALSE;    (* At or beyond high position limit *)
    sysAtPosLimitLow        : BOOL := FALSE;    (* At or beyond low position limit *)
    sysLimitRetractActive   : BOOL := FALSE;    (* Retract limit switch triggered *)
    sysLimitHomeActive      : BOOL := FALSE;    (* Home limit switch triggered *)

    (* Safety status *)
    sysExitGuardActive      : BOOL := FALSE;    (* Piston exit prevention active *)
    sysInCylinderZone       : BOOL := FALSE;    (* Piston is inside cylinder *)

    (* Brake status *)
    sysBrakeEngaged         : BOOL := TRUE;     (* Brake is engaged (spring-applied) *)


    (* ========================================================================
       HANDSHAKE STATUS
       ======================================================================== *)

    sysHandshakeValid       : BOOL := FALSE;    (* Current handshake is valid *)
    sysHandshakeComplete    : BOOL := FALSE;    (* Handshake sequence complete *)
    sysHandshakeTimeout     : BOOL := FALSE;    (* Handshake timed out *)
    sysFaultResetPending    : BOOL := FALSE;    (* Fault reset handshake in progress *)


    (* ========================================================================
       DIAGNOSTICS
       ======================================================================== *)

    sysCycleCount           : UDINT := 0;       (* Main program cycle counter *)
    sysLastFaultCode        : E_FaultCode := FAULT_NONE;    (* Last fault that occurred *)
    sysLastFaultState       : E_SystemState := ST_INIT;     (* State when last fault occurred *)
    sysFaultCount           : UDINT := 0;       (* Total number of faults since power-up *)

    (* Timing diagnostics *)
    sysStateEntryTime       : TIME := T#0MS;    (* Time when current state was entered *)
    sysTimeInState          : TIME := T#0MS;    (* Duration in current state *)

    (* Encoder status *)
    sysEncoderValid         : BOOL := FALSE;    (* Absolute encoder position is valid *)
    sysEncoderBatteryOK     : BOOL := FALSE;    (* Encoder battery status OK *)

END_VAR
