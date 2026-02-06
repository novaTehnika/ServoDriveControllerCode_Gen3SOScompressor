(* ============================================================================
   FaultToInt - Convert Fault Code to Integer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_FaultCode enumeration value to its integer
   representation (0-7) for bit encoding operations.

   Returns: INT (0-7)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to convert *)
*)

CASE Fault OF
    FAULT_NONE:         FaultToInt := 0;
    FAULT_HANDSHAKE:    FaultToInt := 1;
    FAULT_DRIVE:        FaultToInt := 2;
    FAULT_POSITION:     FaultToInt := 3;
    FAULT_HOMING_REQ:   FaultToInt := 4;
    FAULT_PISTON_EXIT:  FaultToInt := 5;
    FAULT_LIMIT_SWITCH: FaultToInt := 6;
    FAULT_ENCODER:      FaultToInt := 7;
ELSE
    FaultToInt := 0;
END_CASE;
