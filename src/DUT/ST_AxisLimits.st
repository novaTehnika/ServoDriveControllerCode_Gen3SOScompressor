(* ============================================================================
   ST_AxisLimits - Axis Limits Structure

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Structure containing position, velocity, and torque limits.
   Used for mode-dependent limit configuration.
   ============================================================================ *)

TYPE ST_AxisLimits :
STRUCT
    (* Position limits (mm) *)
    PosMax          : LREAL;    (* Maximum position limit *)
    PosMin          : LREAL;    (* Minimum position limit *)

    (* Velocity limits (mm/s) *)
    VelMax          : LREAL;    (* Maximum velocity limit *)
    VelMin          : LREAL;    (* Minimum velocity limit (usually negative of VelMax) *)

    (* Torque limits (% of rated) *)
    TorqueMax       : LREAL;    (* Maximum torque limit (positive direction) *)
    TorqueMin       : LREAL;    (* Minimum torque limit (negative direction) *)

    (* Acceleration/Deceleration limits (mm/s^2) *)
    AccelMax        : LREAL;    (* Maximum acceleration *)
    DecelMax        : LREAL;    (* Maximum deceleration *)
END_STRUCT
END_TYPE
