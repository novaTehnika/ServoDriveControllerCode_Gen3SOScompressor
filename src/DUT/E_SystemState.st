(* ============================================================================
   E_SystemState - System State Enumeration

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Enumeration of all possible states in the main state machine.
   States are grouped by function: Initialization, Idle, Transitions,
   Operational, and Fault states.
   ============================================================================ *)

TYPE E_SystemState :
(
    (* ========================================================================
       INITIALIZATION STATES (0-9)
       ======================================================================== *)
    ST_INIT := 0,               (* Power-on initialization *)
    ST_ENCODER_CHECK := 1,      (* Validate absolute encoder status *)
    ST_REQUIRE_ABS_HOME := 2,   (* Encoder invalid, set flag and proceed to idle *)


    (* ========================================================================
       IDLE STATE (10-19)
       ======================================================================== *)
    ST_IDLE := 10,              (* Drive OFF, brake engaged, awaiting commands *)


    (* ========================================================================
       TRANSITION STATES - ACTIVATION (20-29)
       ======================================================================== *)
    ST_DRIVE_ENABLE := 20,      (* Power on drive, wait for ready *)
    ST_BRAKE_RELEASE := 21,     (* Release brake, wait for mechanical delay *)


    (* ========================================================================
       OPERATIONAL STATES (30-59)
       ======================================================================== *)
    ST_BRAKE_HOLD := 30,        (* Mode 001: Drive ON, brake engaged *)
    ST_POSITION_CTRL := 31,     (* Mode 010: Position control active *)
    ST_VELOCITY_CTRL := 32,     (* Mode 011: Velocity control active *)
    ST_TORQUE_CTRL := 33,       (* Mode 100: Torque control active *)
    ST_GO_HOME := 34,           (* Mode 101: Moving to home position *)

    (* Homing states *)
    ST_HOME_LIMIT := 40,        (* Mode 110: Homing to limit switch - uses FB_HomeLimit *)
    ST_HOME_EOT := 41,          (* Mode 111: Homing to end-of-travel - uses FB_HomeEOT *)
    ST_HOME_COMPLETE := 42,     (* Homing finished, awaiting mode change *)

    (* DEPRECATED: Homing sub-states - Limit Switch (Mode 110)
       These states are now handled internally by FB_HomeLimit.
       Values kept for backward compatibility - states redirect to ST_HOME_LIMIT. *)
    ST_HOME_LIM_APPROACH := 43, (* DEPRECATED - use FB_HomeLimit *)
    ST_HOME_LIM_DETECT := 44,   (* DEPRECATED - use FB_HomeLimit *)
    ST_HOME_LIM_BACKOFF := 45,  (* DEPRECATED - use FB_HomeLimit *)
    ST_HOME_LIM_SETREF := 46,   (* DEPRECATED - use FB_HomeLimit *)

    (* DEPRECATED: Homing sub-states - End-of-Travel (Mode 111)
       These states are now handled internally by FB_HomeEOT.
       Values kept for backward compatibility - states redirect to ST_HOME_EOT. *)
    ST_HOME_EOT_FAST := 50,     (* DEPRECATED - use FB_HomeEOT *)
    ST_HOME_EOT_SLOW := 51,     (* DEPRECATED - use FB_HomeEOT *)
    ST_HOME_EOT_DETECT := 52,   (* DEPRECATED - use FB_HomeEOT *)
    ST_HOME_EOT_SETREF := 53,   (* DEPRECATED - use FB_HomeEOT *)


    (* ========================================================================
       TRANSITION STATE - INTER-MODE (60-69)
       ======================================================================== *)
    ST_HOLD_POSITION := 60,     (* MC_Halt active, brake disengaged, awaiting handshake *)


    (* ========================================================================
       TRANSITION STATES - DEACTIVATION (70-79)
       ======================================================================== *)
    ST_BRAKE_ENGAGE := 70,      (* Engage brake, wait for mechanical delay *)
    ST_DRIVE_DISABLE := 71,     (* Power off drive, return to idle *)


    (* ========================================================================
       FAULT STATES (90-99)
       ======================================================================== *)
    ST_FAULT := 90,             (* Fault active, no brake, MC_Halt, await handshake/timeout *)
    ST_FAULT_RECOVERY := 91,    (* Legacy - may be removed *)
    ST_FAULT_IDLE := 92         (* Drive disabled, brake engaged, await fault reset handshake *)

);
END_TYPE
