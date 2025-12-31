(* ============================================================================
   GVL_Position - Global Variable List for Position References

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This GVL contains position landmark references and related position tracking
   variables. These values are established during homing and used for position
   limiting and safety features.

   Coordinate System:
   - Single zero reference established by homing to limit switch (Mode 110)
   - Positive direction: toward cylinder EOT (compression direction)
   - All positions in millimeters (mm)
   ============================================================================ *)

VAR_GLOBAL
    (* ========================================================================
       POSITION LANDMARKS
       These are set during commissioning and/or homing sequences
       ======================================================================== *)

    (* Limit switch positions *)
    posLimitRetract         : LREAL := -5.0;    (* Position of retract limit switch *)
    posLimitHome            : LREAL := 0.0;     (* Position of homing limit switch (= home) *)

    (* Cylinder boundaries *)
    posCylinderEntry        : LREAL := 20.0;    (* Position where piston enters cylinder *)
    posCylinderEOT          : LREAL := 300.0;   (* End-of-travel inside cylinder *)

    (* Software limits (enforced in all modes) *)
    posSoftLimitMin         : LREAL := 0.0;     (* Minimum allowed position (exit prevention) *)
    posSoftLimitMax         : LREAL := 305.0;   (* Maximum allowed position *)


    (* ========================================================================
       HOMING REFERENCE POSITIONS
       These are recorded during homing sequences
       ======================================================================== *)

    (* Mode 110 - Home to Limit Switch *)
    posHomeLimitSwitch      : LREAL := 0.0;     (* Recorded position at limit switch *)

    (* Mode 111 - Home to End-of-Travel *)
    posHomeEOT              : LREAL := 0.0;     (* Recorded position at EOT stall *)

    (* Combined home reference *)
    posHomeOffset           : LREAL := 0.0;     (* Offset between limit switch and EOT *)


    (* ========================================================================
       DYNAMIC POSITION TRACKING
       ======================================================================== *)

    (* Position setpoint for Go Home (Mode 101) *)
    posGoHomeTarget         : LREAL := 0.0;     (* Target position for Go Home move *)

    (* Position at mode entry (for reference) *)
    posModeEntryPosition    : LREAL := 0.0;     (* Position when entering current mode *)

    (* Exit guard boundary *)
    posExitGuardBoundary    : LREAL := 0.0;     (* Dynamic boundary for exit prevention *)


    (* ========================================================================
       POSITION VALIDITY FLAGS
       ======================================================================== *)

    posAbsoluteValid        : BOOL := FALSE;    (* Absolute position from encoder is valid *)
    posHomeLimitValid       : BOOL := FALSE;    (* Home limit switch position is valid *)
    posHomeEOTValid         : BOOL := FALSE;    (* EOT position is valid *)

END_VAR
