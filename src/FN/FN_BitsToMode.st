(* ============================================================================
   FN_BitsToMode - Convert 3 Digital Input Bits to Operating Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts 3 digital input bits to the corresponding E_OperatingMode
   enumeration value. Calculates the integer value from bits then
   maps to the corresponding mode.

   Returns: E_OperatingMode

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR:
       nValue          : INT              (* Calculated integer value
*)

(* Calculate integer value from bits *)
nValue := 0;
IF Bit0 THEN nValue := nValue + 1; END_IF;
IF Bit1 THEN nValue := nValue + 2; END_IF;
IF Bit2 THEN nValue := nValue + 4; END_IF;

(* Convert integer to mode *)
IF nValue = 0 THEN
    FN_BitsToMode := MODE_IDLE;
ELSIF nValue = 1 THEN
    FN_BitsToMode := MODE_BRAKE_HOLD;
ELSIF nValue = 2 THEN
    FN_BitsToMode := MODE_POSITION;
ELSIF nValue = 3 THEN
    FN_BitsToMode := MODE_VELOCITY;
ELSIF nValue = 4 THEN
    FN_BitsToMode := MODE_TORQUE;
ELSIF nValue = 5 THEN
    FN_BitsToMode := MODE_GO_HOME;
ELSIF nValue = 6 THEN
    FN_BitsToMode := MODE_HOME_LIMIT;
ELSIF nValue = 7 THEN
    FN_BitsToMode := MODE_HOME_EOT;
ELSE
    FN_BitsToMode := MODE_IDLE;
END_IF;
