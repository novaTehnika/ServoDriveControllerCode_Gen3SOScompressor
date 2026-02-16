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

IF Mode = MODE_IDLE THEN
    FN_ModeToInt := 0;
ELSIF Mode = MODE_BRAKE_HOLD THEN
    FN_ModeToInt := 1;
ELSIF Mode = MODE_POSITION THEN
    FN_ModeToInt := 2;
ELSIF Mode = MODE_VELOCITY THEN
    FN_ModeToInt := 3;
ELSIF Mode = MODE_TORQUE THEN
    FN_ModeToInt := 4;
ELSIF Mode = MODE_GO_HOME THEN
    FN_ModeToInt := 5;
ELSIF Mode = MODE_HOME_LIMIT THEN
    FN_ModeToInt := 6;
ELSIF Mode = MODE_HOME_EOT THEN
    FN_ModeToInt := 7;
ELSE
    FN_ModeToInt := 0;
END_IF;
 