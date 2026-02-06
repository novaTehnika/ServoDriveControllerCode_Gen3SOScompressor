(* ============================================================================
   FB_DigitalInputFilter - Digital Input Debounce Filter

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Debounces a digital input signal by requiring the input to be stable for
   a configurable time before the output changes.

   Features:
   - Configurable debounce time
   - Edge detection outputs
   - Raw input passthrough for debugging

   Variables (define in MWiec GUI):
     VAR_INPUT:
       RawInput            : BOOL             (* Raw digital input *)
       DebounceTime        : TIME := T#10MS   (* Debounce time *)
     VAR_OUTPUT:
       FilteredOutput      : BOOL             (* Debounced output *)
       RisingEdge          : BOOL             (* Rising edge detected on filtered output *)
       FallingEdge         : BOOL             (* Falling edge detected on filtered output *)
       Stable              : BOOL             (* Output has been stable (not changing) *)
       RawPassthrough      : BOOL             (* Raw input passthrough for debugging *)
     VAR:
       tDebounce           : TON              (* Debounce timer *)
       bLastRaw            : BOOL             (* Previous raw input state *)
       bLastFiltered       : BOOL             (* Previous filtered output state *)
       bPendingState       : BOOL             (* State we're debouncing toward *)
       bFirstScan          : BOOL := TRUE     (* First scan flag for initialization *)
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

(* Passthrough for debugging *)
RawPassthrough := RawInput;

(* First scan initialization - accept current input state immediately *)
IF bFirstScan THEN
    FilteredOutput := RawInput;
    bLastRaw := RawInput;
    bLastFiltered := RawInput;
    bPendingState := RawInput;
    Stable := TRUE;
    RisingEdge := FALSE;
    FallingEdge := FALSE;
    bFirstScan := FALSE;
    RETURN;
END_IF;

(* Detect raw input change *)
IF RawInput <> bLastRaw THEN
    (* Input changed - start debouncing toward new state *)
    bPendingState := RawInput;
    tDebounce(IN := FALSE);  (* Reset timer *)
END_IF;

bLastRaw := RawInput;

(* Run debounce timer if pending state differs from current output *)
IF bPendingState <> FilteredOutput THEN
    tDebounce(IN := TRUE, PT := DebounceTime);
    Stable := FALSE;

    IF tDebounce.Q THEN
        (* Debounce complete - update output *)
        FilteredOutput := bPendingState;
        tDebounce(IN := FALSE);
        Stable := TRUE;
    END_IF;
ELSE
    (* No state change pending *)
    tDebounce(IN := FALSE);
    Stable := TRUE;
END_IF;

(* Edge detection on filtered output *)
RisingEdge := FilteredOutput AND NOT bLastFiltered;
FallingEdge := NOT FilteredOutput AND bLastFiltered;
bLastFiltered := FilteredOutput;
