(* ============================================================================
   FaultMatchesModeBits - Cross-Type Bit Comparison

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Checks if a fault code's integer value matches a mode's integer value.
   Used for fault reset mirroring: master mirrors the fault code on DI0-DI2
   using mode encoding, and this function verifies the match.

   Both E_FaultCode and E_OperatingMode use 3-bit encoding (0-7),
   so we compare their integer values directly.

   Returns: BOOL (TRUE if values match)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to compare
       Mode            : E_OperatingMode   (* Mode bits to compare against
*)

(* Both use 3-bit encoding 0-7, so we just compare integer values *)
FaultMatchesModeBits := (FaultToInt(Fault) = ModeToInt(Mode));
