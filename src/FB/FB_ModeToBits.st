(* ============================================================================
   FB_ModeToBits - Convert Operating Mode to 3 Output Bits

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_OperatingMode enumeration value to 3 digital output bits.
   Uses FN_ModeToInt to get the integer value, then extracts individual bits.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Operating mode to convert 
     VAR_OUTPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR:
       nValue          : INT              (* Integer mode value
*)

nValue := FN_ModeToInt(Mode);
Bit0 := (INT_TO_BYTE(nValue) AND INT_TO_BYTE(1)) <> 0;
Bit1 := (INT_TO_BYTE(nValue) AND INT_TO_BYTE(2)) <> 0;
Bit2 := (INT_TO_BYTE(nValue) AND INT_TO_BYTE(4)) <> 0;