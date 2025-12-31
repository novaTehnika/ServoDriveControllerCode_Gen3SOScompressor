(* ============================================================================
   EnumConversions - Centralized Enum Conversion Functions

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Centralized functions for converting between enumerations and their
   integer/bit representations. All enum conversion logic should be here
   to ensure consistency across the codebase.
   ============================================================================ *)


(* ============================================================================
   MODE CONVERSIONS
   ============================================================================ *)

(* Convert E_OperatingMode to integer value (0-7) *)
FUNCTION ModeToInt : INT
VAR_INPUT
    Mode : E_OperatingMode;
END_VAR

CASE Mode OF
    MODE_IDLE:       ModeToInt := 0;
    MODE_BRAKE_HOLD: ModeToInt := 1;
    MODE_POSITION:   ModeToInt := 2;
    MODE_VELOCITY:   ModeToInt := 3;
    MODE_TORQUE:     ModeToInt := 4;
    MODE_GO_HOME:    ModeToInt := 5;
    MODE_HOME_LIMIT: ModeToInt := 6;
    MODE_HOME_EOT:   ModeToInt := 7;
ELSE
    ModeToInt := 0;
END_CASE

END_FUNCTION


(* Convert integer value (0-7) to E_OperatingMode *)
FUNCTION IntToMode : E_OperatingMode
VAR_INPUT
    Value : INT;
END_VAR

CASE Value OF
    0: IntToMode := MODE_IDLE;
    1: IntToMode := MODE_BRAKE_HOLD;
    2: IntToMode := MODE_POSITION;
    3: IntToMode := MODE_VELOCITY;
    4: IntToMode := MODE_TORQUE;
    5: IntToMode := MODE_GO_HOME;
    6: IntToMode := MODE_HOME_LIMIT;
    7: IntToMode := MODE_HOME_EOT;
ELSE
    IntToMode := MODE_IDLE;
END_CASE

END_FUNCTION


(* Convert 3 digital input bits to E_OperatingMode *)
FUNCTION BitsToMode : E_OperatingMode
VAR_INPUT
    Bit0 : BOOL;    (* LSB *)
    Bit1 : BOOL;
    Bit2 : BOOL;    (* MSB *)
END_VAR
VAR
    nValue : INT;
END_VAR

nValue := 0;
IF Bit0 THEN nValue := nValue + 1; END_IF
IF Bit1 THEN nValue := nValue + 2; END_IF
IF Bit2 THEN nValue := nValue + 4; END_IF

BitsToMode := IntToMode(nValue);

END_FUNCTION


(* Convert E_OperatingMode to 3 output bits *)
FUNCTION ModeToBits
VAR_INPUT
    Mode : E_OperatingMode;
END_VAR
VAR_OUTPUT
    Bit0 : BOOL;    (* LSB *)
    Bit1 : BOOL;
    Bit2 : BOOL;    (* MSB *)
END_VAR
VAR
    nValue : INT;
END_VAR

nValue := ModeToInt(Mode);
Bit0 := (nValue AND 1) <> 0;
Bit1 := (nValue AND 2) <> 0;
Bit2 := (nValue AND 4) <> 0;

END_FUNCTION


(* ============================================================================
   FAULT CODE CONVERSIONS
   ============================================================================ *)

(* Convert E_FaultCode to integer value (0-7) *)
FUNCTION FaultToInt : INT
VAR_INPUT
    Fault : E_FaultCode;
END_VAR

CASE Fault OF
    FAULT_NONE:         FaultToInt := 0;
    FAULT_HANDSHAKE:    FaultToInt := 1;
    FAULT_DRIVE:        FaultToInt := 2;
    FAULT_POSITION:     FaultToInt := 3;
    FAULT_HOMING_REQ:   FaultToInt := 4;
    FAULT_PISTON_EXIT:  FaultToInt := 5;
    FAULT_LIMIT_SWITCH: FaultToInt := 6;
    FAULT_ENCODER:      FaultToInt := 7;
ELSE
    FaultToInt := 0;
END_CASE

END_FUNCTION


(* Convert integer value (0-7) to E_FaultCode *)
FUNCTION IntToFault : E_FaultCode
VAR_INPUT
    Value : INT;
END_VAR

CASE Value OF
    0: IntToFault := FAULT_NONE;
    1: IntToFault := FAULT_HANDSHAKE;
    2: IntToFault := FAULT_DRIVE;
    3: IntToFault := FAULT_POSITION;
    4: IntToFault := FAULT_HOMING_REQ;
    5: IntToFault := FAULT_PISTON_EXIT;
    6: IntToFault := FAULT_LIMIT_SWITCH;
    7: IntToFault := FAULT_ENCODER;
ELSE
    IntToFault := FAULT_NONE;
END_CASE

END_FUNCTION


(* Convert E_FaultCode to 3 output bits *)
FUNCTION FaultToBits
VAR_INPUT
    Fault : E_FaultCode;
END_VAR
VAR_OUTPUT
    Bit0 : BOOL;    (* LSB *)
    Bit1 : BOOL;
    Bit2 : BOOL;    (* MSB *)
END_VAR
VAR
    nValue : INT;
END_VAR

nValue := FaultToInt(Fault);
Bit0 := (nValue AND 1) <> 0;
Bit1 := (nValue AND 2) <> 0;
Bit2 := (nValue AND 4) <> 0;

END_FUNCTION


(* ============================================================================
   CROSS-TYPE COMPARISONS
   ============================================================================ *)

(* Check if a fault code value matches mode bits (for fault reset mirroring) *)
FUNCTION FaultMatchesModeBits : BOOL
VAR_INPUT
    Fault : E_FaultCode;
    Mode  : E_OperatingMode;
END_VAR

(* Both use 3-bit encoding 0-7, so we just compare integer values *)
FaultMatchesModeBits := (FaultToInt(Fault) = ModeToInt(Mode));

END_FUNCTION
