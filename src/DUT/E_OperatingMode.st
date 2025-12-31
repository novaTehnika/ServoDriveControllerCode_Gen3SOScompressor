(* ============================================================================
   E_OperatingMode - Operating Mode Enumeration

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Enumeration of operating modes corresponding to the 3-bit mode encoding
   from the master controller.

   Encoding:
   - Bits come from DI0 (LSB), DI1, DI2 (MSB)
   - Mode value = DI2*4 + DI1*2 + DI0
   ============================================================================ *)

TYPE E_OperatingMode :
(
    MODE_IDLE := 0,             (* 000: Idle - Drive OFF, brake engaged *)
    MODE_BRAKE_HOLD := 1,       (* 001: Brake Hold - Drive ON, brake engaged *)
    MODE_POSITION := 2,         (* 010: Position Control - Analog reference *)
    MODE_VELOCITY := 3,         (* 011: Velocity Control - Analog reference *)
    MODE_TORQUE := 4,           (* 100: Torque Control - Analog reference *)
    MODE_GO_HOME := 5,          (* 101: Go Home - Move to home position *)
    MODE_HOME_LIMIT := 6,       (* 110: Home to Limit Switch *)
    MODE_HOME_EOT := 7          (* 111: Home to End-of-Travel (torque detection) *)
);
END_TYPE
