(* ============================================================================
   E_FaultCode - Fault Code Enumeration

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Enumeration of fault codes. When doFaultActive is HIGH, the 3-bit fault
   code is output on DO0-DO2 instead of mode confirmation.

   Encoding:
   - DO0 = bit 0 (LSB), DO1 = bit 1, DO2 = bit 2 (MSB)
   - Fault value = DO2*4 + DO1*2 + DO0
   ============================================================================ *)

TYPE E_FaultCode :
(
    FAULT_NONE := 0,            (* 000: No fault / fault cleared *)
    FAULT_HANDSHAKE := 1,       (* 001: Handshake timeout or mismatch *)
    FAULT_DRIVE := 2,           (* 010: Drive fault (from servo amplifier) *)
    FAULT_POSITION := 3,        (* 011: Position limit exceeded *)
    FAULT_HOMING_REQ := 4,      (* 100: Homing requirement not met *)
    FAULT_PISTON_EXIT := 5,     (* 101: Piston exit prevention triggered *)
    FAULT_LIMIT_SWITCH := 6,    (* 110: Limit switch fault *)
    FAULT_ENCODER := 7          (* 111: Encoder / absolute position fault *)
);
END_TYPE
