(* ============================================================================
   FB_OutputMux - Digital Output Multiplexer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Multiplexes DO0-DO2 outputs between mode confirmation and fault code:
   - Normal mode: DO0-DO2 = Mode confirmation bits
   - Fault mode:  DO0-DO2 = Fault code bits

   Uses centralized conversion functions from FN_FaultToBits.

   Fault Codes (3-bit):
   000 = No fault / cleared
   001 = Handshake timeout/mismatch
   010 = Drive fault
   011 = Position limit exceeded
   100 = Homing requirement not met
   101 = Piston exit prevention triggered
   110 = Limit switch fault
   111 = Encoder/absolute position fault

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       FaultActive         : BOOL             (* Fault state is active
       FaultCode           : E_FaultCode      (* Current fault code
       ModeConfirmBit0     : BOOL             (* Mode confirmation bit 0
       ModeConfirmBit1     : BOOL             (* Mode confirmation bit 1
       ModeConfirmBit2     : BOOL             (* Mode confirmation bit 2
     VAR_OUTPUT:
       DO0_Output          : BOOL             (* Output for DO0
       DO1_Output          : BOOL             (* Output for DO1
       DO2_Output          : BOOL             (* Output for DO2
       FaultActiveOutput   : BOOL             (* Output for doFaultActive (DO5)
     VAR:
       fbFaultToBits       : FB_FaultToBits   (* Fault code to bits converter
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* All outputs low when disabled *)
    DO0_Output := FALSE;
    DO1_Output := FALSE;
    DO2_Output := FALSE;
    FaultActiveOutput := FALSE;
    RETURN;
END_IF;

(* Set fault active output *)
FaultActiveOutput := FaultActive;

IF FaultActive THEN
    (* Fault mode: Output fault code on DO0-DO2 *)
    (* Use centralized conversion function *)
    fbFaultToBits(Fault := FaultCode);
    DO0_Output := fbFaultToBits.Bit0;
    DO1_Output := fbFaultToBits.Bit1;
    DO2_Output := fbFaultToBits.Bit2;
ELSE
    (* Normal mode: Output mode confirmation bits *)
    DO0_Output := ModeConfirmBit0;
    DO1_Output := ModeConfirmBit1;
    DO2_Output := ModeConfirmBit2;
END_IF;
