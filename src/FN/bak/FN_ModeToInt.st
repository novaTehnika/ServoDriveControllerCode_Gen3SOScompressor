(* ============================================================================
   FN_ModeToInt - Convert Operating Mode to Integer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_OperatingMode enumeration value to its integer
   representation (0-7) for bit encoding operations.

   Returns: INT (0-7)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Operating mode to convert
*)

IF Mode = E_OperatingMode#MODE_IDLE THEN
    FN_ModeToInt := 0;
ELSIF Mode = E_OperatingMode#MODE_BRAKE_HOLD THEN
    FN_ModeToInt := 1;
ELSIF Mode = E_OperatingMode#MODE_POSITION THEN
    FN_ModeToInt := 2;
ELSIF Mode = E_OperatingMode#MODE_VELOCITY THEN
    FN_ModeToInt := 3;
ELSIF Mode = E_OperatingMode#MODE_TORQUE THEN
    FN_ModeToInt := 4;
ELSIF Mode = E_OperatingMode#MODE_GO_HOME THEN
    FN_ModeToInt := 5;
ELSIF Mode = E_OperatingMode#MODE_HOME_LIMIT THEN
    FN_ModeToInt := 6;
ELSIF Mode = E_OperatingMode#MODE_HOME_EOT THEN
    FN_ModeToInt := 7;
ELSE
    FN_ModeToInt := 0;
END_IF;
