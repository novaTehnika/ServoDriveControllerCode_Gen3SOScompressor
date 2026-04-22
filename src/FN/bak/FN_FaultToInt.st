(* ============================================================================
   FN_FaultToInt - Convert Fault Code to Integer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_FaultCode enumeration value to its integer
   representation (0-7) for bit encoding operations.

   Returns: INT (0-7)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to convert
*)

IF Fault = E_FaultCode#FAULT_NONE THEN
    FN_FaultToInt := 0;
ELSIF Fault = E_FaultCode#FAULT_HANDSHAKE THEN
    FN_FaultToInt := 1;
ELSIF Fault = E_FaultCode#FAULT_DRIVE THEN
    FN_FaultToInt := 2;
ELSIF Fault = E_FaultCode#FAULT_POSITION THEN
    FN_FaultToInt := 3;
ELSIF Fault = E_FaultCode#FAULT_HOMING_REQ THEN
    FN_FaultToInt := 4;
ELSIF Fault = E_FaultCode#FAULT_PISTON_EXIT THEN
    FN_FaultToInt := 5;
ELSIF Fault = E_FaultCode#FAULT_LIMIT_SWITCH THEN
    FN_FaultToInt := 6;
ELSIF Fault = E_FaultCode#FAULT_ENCODER THEN
    FN_FaultToInt := 7;
ELSE
    FN_FaultToInt := 0;
END_IF;
