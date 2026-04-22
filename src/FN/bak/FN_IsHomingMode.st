(* ============================================================================
   FN_IsHomingMode - Check if Mode is a Homing Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Checks if the given operating mode is one of the homing modes
   (Go Home, Home to Limit Switch, or Home to End-of-Travel).

   Returns: BOOL (TRUE if mode is a homing mode)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Mode to check
*)

FN_IsHomingMode := (Mode = E_OperatingMode#MODE_GO_HOME) OR
                (Mode = E_OperatingMode#MODE_HOME_LIMIT) OR
                (Mode = E_OperatingMode#MODE_HOME_EOT);
