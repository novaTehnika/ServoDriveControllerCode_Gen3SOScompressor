(* ============================================================================
   IntToMode - Convert Integer to Operating Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an integer value (0-7) to the corresponding E_OperatingMode
   enumeration value. Out-of-range values default to MODE_IDLE.

   Returns: E_OperatingMode

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Value           : INT              (* Integer value 0-7 *)
*)

CASE Value OF
    0: IntToMode := MODE_IDLE;
    1: IntToMode := MODE_BRAKE_HOLD;
    2: IntToMode := MODE_POSITION;
    3: IntToMode := MODE_VELOCITY;
    4: IntToMode := MODE_TORQUE;
    5: IntToMode := MODE_GO_HOME;
    6: IntToMode := MODE_HOME_LIMIT;
    7: IntToMode := MODE_HOME_EOT;
ELSE
    IntToMode := MODE_IDLE;
END_CASE;
