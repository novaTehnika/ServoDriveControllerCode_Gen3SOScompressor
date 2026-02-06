(* ============================================================================
   ModeToInt - Convert Operating Mode to Integer

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
    ModeToInt := 0;
ELSIF Mode = MODE_BRAKE_HOLD THEN
    ModeToInt := 1;
ELSIF Mode = MODE_POSITION THEN
    ModeToInt := 2;
ELSIF Mode = MODE_VELOCITY THEN
    ModeToInt := 3;
ELSIF Mode = MODE_TORQUE THEN
    ModeToInt := 4;
ELSIF Mode = MODE_GO_HOME THEN
    ModeToInt := 5;
ELSIF Mode = MODE_HOME_LIMIT THEN
    ModeToInt := 6;
ELSIF Mode = MODE_HOME_EOT THEN
    ModeToInt := 7;
ELSE
    ModeToInt := 0;
END_IF;
 