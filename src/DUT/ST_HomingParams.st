(* ============================================================================
   ST_HomingParams - Homing Parameters Structure

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Structure containing parameters for homing sequences.
   Supports both limit switch homing (Mode 110) and EOT homing (Mode 111).
   ============================================================================ *)

TYPE ST_HomingParams :
STRUCT
    (* Approach velocities (mm/s) *)
    VelFast         : LREAL;    (* Fast approach velocity *)
    VelSlow         : LREAL;    (* Slow approach velocity *)

    (* Torque parameters (% of rated) *)
    TorqueThreshold : LREAL;    (* Stall detection threshold (EOT homing) *)
    TorqueLimit     : LREAL;    (* Torque limit during slow approach *)

    (* Distance parameters (mm) *)
    ApproachDist    : LREAL;    (* Distance for slow approach phase *)
    BackoffDist     : LREAL;    (* Distance to back off after detection *)

    (* Position to set after homing (mm) *)
    SetPosition     : LREAL;    (* Position value to assign at home point *)

    (* Timing *)
    StallDetectTime : TIME;     (* Time to confirm stall condition *)
    Timeout         : TIME;     (* Overall homing sequence timeout *)

    (* Direction *)
    Direction       : INT;      (* Homing direction: 1 = positive, -1 = negative *)
END_STRUCT
END_TYPE
