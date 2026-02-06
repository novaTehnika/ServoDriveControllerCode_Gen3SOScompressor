(* ============================================================================
   FaultToBits - Convert Fault Code to 3 Output Bits

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_FaultCode enumeration value to 3 digital output bits.
   Uses FaultToInt to get the integer value, then extracts individual bits.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to convert
     VAR_OUTPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR:
       nValue          : INT              (* Integer fault value
*)

nValue := FaultToInt(Fault);
Bit0 := (nValue AND 1) <> 0;
Bit1 := (nValue AND 2) <> 0;
Bit2 := (nValue AND 4) <> 0;
