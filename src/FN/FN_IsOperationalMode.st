(* ============================================================================
   FN_IsOperationalMode - Check if Mode Requires Homing

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Checks if the given operating mode is an operational mode that requires
   homing to have been completed before entry. Operational modes are those
   that perform motion control using absolute position references.

   Returns: BOOL (TRUE if mode requires homing)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Mode to check
*)

FN_IsOperationalMode := (Mode = E_OperatingMode#MODE_BRAKE_HOLD) OR
                     (Mode = E_OperatingMode#MODE_POSITION) OR
                     (Mode = E_OperatingMode#MODE_VELOCITY) OR
                     (Mode = E_OperatingMode#MODE_TORQUE);
