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

CASE Mode OF
    MODE_IDLE:       ModeToInt := 0;
    MODE_BRAKE_HOLD: ModeToInt := 1;
    MODE_POSITION:   ModeToInt := 2;
    MODE_VELOCITY:   ModeToInt := 3;
    MODE_TORQUE:     ModeToInt := 4;
    MODE_GO_HOME:    ModeToInt := 5;
    MODE_HOME_LIMIT: ModeToInt := 6;
    MODE_HOME_EOT:   ModeToInt := 7;
ELSE
    ModeToInt := 0;
END_CASE;
 