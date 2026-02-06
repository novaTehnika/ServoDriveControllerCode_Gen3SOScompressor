(* ============================================================================
   BitsToMode - Convert 3 Digital Input Bits to Operating Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts 3 digital input bits to the corresponding E_OperatingMode
   enumeration value. Calculates the integer value from bits then
   calls IntToMode for the conversion.

   Returns: E_OperatingMode

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Bit0            : BOOL             (* LSB *)
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB *)
     VAR:
       nValue          : INT              (* Calculated integer value *)
*)

nValue := 0;
IF Bit0 THEN nValue := nValue + 1; END_IF;
IF Bit1 THEN nValue := nValue + 2; END_IF;
IF Bit2 THEN nValue := nValue + 4; END_IF;

BitsToMode := IntToMode(nValue);
