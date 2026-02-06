(* ============================================================================
   FB_PositionOutput - Two-Stage Position Output Signal Mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Maps the actual position to analog output voltage using a two-stage
   scaling scheme for improved resolution in the critical in-cylinder region.
   This signal is sent to the master controller.

   Two-Stage Mapping:
   - Stage 1: In-cylinder region (higher resolution)
     Position 0-200mm maps to -10V to +5V (15V range for 200mm)
   - Stage 2: Out-of-cylinder region (lower resolution)
     Position 200-305mm maps to +5V to +10V (5V range for 105mm)

   This provides approximately 3x better resolution in the working region
   where the piston is inside the cylinder.

   Output:
   - Raw INT: -32768 to +32767 corresponds to -10V to +10V

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the mapper *)
       ActualPosition      : LREAL            (* Current position in mm *)
       Stage1PosMin        : LREAL            (* Stage 1 position start (mm) *)
       Stage1PosMax        : LREAL            (* Stage 1 position end (mm) *)
       Stage1VoltMin       : LREAL            (* Stage 1 voltage start (V) *)
       Stage1VoltMax       : LREAL            (* Stage 1 voltage end (V) *)
       Stage2PosMin        : LREAL            (* Stage 2 position start (mm) *)
       Stage2PosMax        : LREAL            (* Stage 2 position end (mm) *)
       Stage2VoltMin       : LREAL            (* Stage 2 voltage start (V) *)
       Stage2VoltMax       : LREAL            (* Stage 2 voltage end (V) *)
     VAR_OUTPUT:
       RawAnalogOutput     : INT              (* Raw analog output value *)
       OutputVoltage       : LREAL            (* Calculated output voltage *)
       InStage1            : BOOL             (* Position is in stage 1 region *)
       InStage2            : BOOL             (* Position is in stage 2 region *)
     VAR:
       rVoltage            : LREAL            (* Calculated voltage *)
       rNormalized         : LREAL            (* Normalized position within stage *)
       rRawValue           : LREAL            (* Raw value before conversion *)
     VAR CONSTANT:
       RAW_MIN             : LREAL := -32768.0
       RAW_MAX             : LREAL := 32767.0
       VOLT_MIN            : LREAL := -10.0
       VOLT_MAX            : LREAL := 10.0
       VOLT_RANGE          : LREAL := 20.0
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    RawAnalogOutput := 0;
    OutputVoltage := 0.0;
    InStage1 := FALSE;
    InStage2 := FALSE;
    RETURN;
END_IF;

(* Determine which stage the position falls into *)
IF ActualPosition <= Stage1PosMax THEN
    (* Stage 1: In-cylinder region *)
    InStage1 := TRUE;
    InStage2 := FALSE;

    (* Calculate normalized position within stage 1 *)
    IF (Stage1PosMax - Stage1PosMin) > 0.0 THEN
        rNormalized := (ActualPosition - Stage1PosMin) / (Stage1PosMax - Stage1PosMin);
    ELSE
        rNormalized := 0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rNormalized := LIMIT(0.0, rNormalized, 1.0);

    (* Calculate voltage for stage 1 *)
    rVoltage := Stage1VoltMin + (rNormalized * (Stage1VoltMax - Stage1VoltMin));

ELSE
    (* Stage 2: Out-of-cylinder region *)
    InStage1 := FALSE;
    InStage2 := TRUE;

    (* Calculate normalized position within stage 2 *)
    IF (Stage2PosMax - Stage2PosMin) > 0.0 THEN
        rNormalized := (ActualPosition - Stage2PosMin) / (Stage2PosMax - Stage2PosMin);
    ELSE
        rNormalized := 0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rNormalized := LIMIT(0.0, rNormalized, 1.0);

    (* Calculate voltage for stage 2 *)
    rVoltage := Stage2VoltMin + (rNormalized * (Stage2VoltMax - Stage2VoltMin));
END_IF;

(* Clamp voltage to valid range *)
rVoltage := LIMIT(VOLT_MIN, rVoltage, VOLT_MAX);
OutputVoltage := rVoltage;

(* Convert voltage to raw output value *)
(* Voltage -10V to +10V maps to raw -32768 to +32767 *)
rRawValue := ((rVoltage - VOLT_MIN) / VOLT_RANGE) * (RAW_MAX - RAW_MIN) + RAW_MIN;

(* Convert to integer with clamping *)
IF rRawValue < RAW_MIN THEN
    RawAnalogOutput := -32768;
ELSIF rRawValue > RAW_MAX THEN
    RawAnalogOutput := 32767;
ELSE
    RawAnalogOutput := LREAL_TO_INT(rRawValue);
END_IF;
