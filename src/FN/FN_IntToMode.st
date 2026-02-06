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
       Value           : INT              (* Integer value 0-7
*)

IF Value = 0 THEN
    IntToMode := MODE_IDLE;
ELSIF Value = 1 THEN
    IntToMode := MODE_BRAKE_HOLD;
ELSIF Value = 2 THEN
    IntToMode := MODE_POSITION;
ELSIF Value = 3 THEN
    IntToMode := MODE_VELOCITY;
ELSIF Value = 4 THEN
    IntToMode := MODE_TORQUE;
ELSIF Value = 5 THEN
    IntToMode := MODE_GO_HOME;
ELSIF Value = 6 THEN
    IntToMode := MODE_HOME_LIMIT;
ELSIF Value = 7 THEN
    IntToMode := MODE_HOME_EOT;
ELSE
    IntToMode := MODE_IDLE;
END_IF;
