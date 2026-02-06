(* ============================================================================
   IntToFault - Convert Integer to Fault Code

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an integer value (0-7) to the corresponding E_FaultCode
   enumeration value. Out-of-range values default to FAULT_NONE.

   Returns: E_FaultCode

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Value           : INT              (* Integer value 0-7
*)

CASE Value OF
    0: IntToFault := FAULT_NONE;
    1: IntToFault := FAULT_HANDSHAKE;
    2: IntToFault := FAULT_DRIVE;
    3: IntToFault := FAULT_POSITION;
    4: IntToFault := FAULT_HOMING_REQ;
    5: IntToFault := FAULT_PISTON_EXIT;
    6: IntToFault := FAULT_LIMIT_SWITCH;
    7: IntToFault := FAULT_ENCODER;
ELSE
    IntToFault := FAULT_NONE;
END_CASE;
