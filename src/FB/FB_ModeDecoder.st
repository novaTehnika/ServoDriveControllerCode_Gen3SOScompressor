(* ============================================================================
   FB_ModeDecoder - Mode Command Decoder

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Decodes the 3-bit mode command from digital inputs (DI0, DI1, DI2) into
   an E_OperatingMode enumeration value.

   NOTE: Input debouncing is handled externally by FB_DigitalInputFilter.
   This function block expects pre-filtered (debounced) inputs.

   Inputs:
   - ModeBit0, ModeBit1, ModeBit2: Filtered digital input states

   Outputs:
   - DecodedMode: The operating mode enumeration
   - ModeValue: The numeric mode value (0-7)
   - ModeStable: TRUE when all input bits have the same value as previous scan
   ============================================================================ *)

FUNCTION_BLOCK FB_ModeDecoder
VAR_INPUT
    Enable          : BOOL;             (* Enable the decoder *)
    ModeBit0        : BOOL;             (* Mode bit 0 - LSB (DI0) - pre-filtered *)
    ModeBit1        : BOOL;             (* Mode bit 1 (DI1) - pre-filtered *)
    ModeBit2        : BOOL;             (* Mode bit 2 - MSB (DI2) - pre-filtered *)
END_VAR

VAR_OUTPUT
    DecodedMode     : E_OperatingMode;  (* Decoded operating mode *)
    ModeValue       : INT;              (* Numeric mode value 0-7 *)
    ModeStable      : BOOL;             (* Mode has been stable (no change from last scan) *)
    ModeChanged     : BOOL;             (* Rising edge - mode just changed *)
END_VAR

VAR
    nCurrentValue   : INT;              (* Current mode value *)
    nPreviousValue  : INT;              (* Previous scan mode value *)
    bPrevStable     : BOOL;             (* Previous stable state *)
END_VAR

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset outputs when disabled *)
    DecodedMode := MODE_IDLE;
    ModeValue := 0;
    ModeStable := FALSE;
    ModeChanged := FALSE;
    nPreviousValue := 0;
    RETURN;
END_IF

(* Decode mode from bits using centralized conversion function *)
DecodedMode := BitsToMode(ModeBit0, ModeBit1, ModeBit2);
nCurrentValue := ModeToInt(DecodedMode);

(* Update outputs *)
ModeValue := nCurrentValue;

(* Mode is stable if value matches previous scan *)
ModeStable := (nCurrentValue = nPreviousValue);

(* Detect mode change (value different from previous scan) *)
ModeChanged := (nCurrentValue <> nPreviousValue);

(* Save for next scan *)
nPreviousValue := nCurrentValue;
bPrevStable := ModeStable;

END_FUNCTION_BLOCK
