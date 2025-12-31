(* ============================================================================
   GVL_IO - Global Variable List for I/O Mappings

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This GVL contains all digital and analog I/O variable declarations with
   direct hardware address mappings for the MP2600iec controller.

   Note: Actual I/O addresses (%IX, %QX, %IW, %QW) must be verified against
   the hardware configuration in MotionWorksIEC Express.
   ============================================================================ *)

VAR_GLOBAL
    (* ========================================================================
       DIGITAL INPUTS (8 Available)
       PNP limit switches: TRUE = not triggered, FALSE = triggered (fail-safe)
       ======================================================================== *)

    (* Mode Command from Master (3-bit) *)
    diModeBit0          AT %IX0.0 : BOOL;   (* Mode bit 0 - LSB *)
    diModeBit1          AT %IX0.1 : BOOL;   (* Mode bit 1 *)
    diModeBit2          AT %IX0.2 : BOOL;   (* Mode bit 2 - MSB *)

    (* Motion Control *)
    diMotionEnable      AT %IX0.3 : BOOL;   (* Motion enable from master *)

    (* Limit Switches - Tolomatic #8100-9092, Solid State, NC, PNP *)
    (* TRUE = switch NOT triggered (normal), FALSE = triggered or wire broken *)
    diLimitRetract      AT %IX0.4 : BOOL;   (* Retracted position limit switch *)
    diLimitHome         AT %IX0.5 : BOOL;   (* Homing reference limit switch *)

    (* Fault Control *)
    diFaultReset        AT %IX0.6 : BOOL;   (* Fault reset command from master *)

    (* Reserved *)
    diReserved          AT %IX0.7 : BOOL;   (* Reserved for future use *)


    (* ========================================================================
       DIGITAL OUTPUTS (8 Available)
       ======================================================================== *)

    (* Mode Confirmation to Master (3-bit) / Fault Code when doFaultActive=TRUE *)
    doModeConfBit0      AT %QX0.0 : BOOL;   (* Mode confirm bit 0 / Fault code bit 0 *)
    doModeConfBit1      AT %QX0.1 : BOOL;   (* Mode confirm bit 1 / Fault code bit 1 *)
    doModeConfBit2      AT %QX0.2 : BOOL;   (* Mode confirm bit 2 / Fault code bit 2 *)

    (* Brake Control *)
    (* Spring-engaged, electrically released: HIGH = brake disengaged (motion allowed) *)
    doBrakeDisengage    AT %QX0.3 : BOOL;   (* Brake release command *)

    (* Status Outputs *)
    doPerformanceStatus AT %QX0.4 : BOOL;   (* Mode-dependent status indicator *)
    doFaultActive       AT %QX0.5 : BOOL;   (* Fault indicator: HIGH = fault active *)
    doInMotion          AT %QX0.6 : BOOL;   (* Axis in motion indicator *)
    doHomingComplete    AT %QX0.7 : BOOL;   (* Homing sequence complete *)


    (* ========================================================================
       ANALOG INPUTS (1 Available)
       Range: -10V to +10V mapped to INT -32768 to +32767
       Resolution: 20V / 65536 = 0.305 mV/count
       ======================================================================== *)

    aiReference         AT %IW0   : INT;    (* Mode-dependent reference signal *)
                                            (* Position mode: position reference *)
                                            (* Velocity mode: velocity reference *)
                                            (* Torque mode: torque reference *)


    (* ========================================================================
       ANALOG OUTPUTS (1 Available)
       Range: -10V to +10V mapped to INT -32768 to +32767
       ======================================================================== *)

    aoPositionOutput    AT %QW0   : INT;    (* Position output signal to master *)
                                            (* Uses two-stage scaling for resolution *)

END_VAR
