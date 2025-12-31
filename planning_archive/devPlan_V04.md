# Implementation Plan: MP2600iec Servo Controller for Gen3 SOS Compressor
## Revision 3 - Incorporating V02 Feedback

## Overview
Develop IEC 61131-3 Structured Text code for a Yaskawa MP2600iec servo controller operating as a slave to a Simulink Desktop Real Time master. The application controls a linear actuator for a medical device that produces oxygen micro-bubble saline solutions.

## Hardware Summary
| Component | Specification |
|-----------|---------------|
| Controller | Yaskawa MP2600iec (MotionWorksIEC Express) |
| Drive | Yaskawa Sigma-7 (SGD7S2R8FE0A000300) |
| Motor | 400W Sigma-7 with brake and absolute encoder (SGM7J-04A6A6C) |
| Actuator | Tolomatic RSA64 (~305mm usable stroke) |
| Gearing | 70:1 motor-to-leadscrew, 5.08mm per leadscrew revolution |
| Limit Switches | Tolomatic #8100-9092, Solid State, Normally Closed, PNP |
| I/O | 8 DI, 8 DO, 1 AI (-10V to +10V), 1 AO |
| Master Interface | NI PCI 6251 DAQ via analog/digital signals |

## Operating Modes (3-bit encoding)
| Code | Mode | Description |
|------|------|-------------|
| 000 | Idle | Drive power OFF, brake engaged |
| 001 | Brake Hold | Drive powered, brake engaged, holding position |
| 010 | Position Control | Analog input as position reference |
| 011 | Velocity Control | Analog input as velocity reference |
| 100 | Torque Control | Analog input as torque reference |
| 101 | Go Home | Move to home position (limit switch location) |
| 110 | Home to Limit Switch | Homing sequence using physical limit switch |
| 111 | Home to End-of-Travel | Homing via torque-limited stall detection (extend direction) |

### Mode 101 (Go Home) Behavior
- **If homing requirements ARE met**: Execute position move to home (limit switch) position, hold there until mode change
- **If homing requirements NOT met**: Execute Home to Limit Switch (Mode 110) instead, which satisfies the requirement
- Does NOT modify position variables/settings unless homing requirement was previously unmet

## Homing Requirements
| Requirement | When Required | Cleared By |
|-------------|---------------|------------|
| Absolute Encoder Home | Encoder invalid (battery failure) | Mode 110 (Home to Limit Switch) |
| EOT Home | Every power-up | Mode 111 (Home to End-of-Travel) |

**Enforcement Pattern**:
1. If operational mode (001-100) commanded while homing requirements unmet → enter Homing Fault
2. Fault cleared via handshake (diFaultReset + diMotionEnable LOW)
3. If re-entering operational mode with requirements still unmet → re-enter fault
4. Master must command homing modes (110/111) or Go Home (101) to satisfy requirements
5. Slave enforces requirements but requires master instruction for all motion

## Digital I/O Mapping

### Digital Inputs (8)
| Pin | Signal | Description |
|-----|--------|-------------|
| DI0 | `diModeBit0` | Mode command bit 0 (LSB) from NI DAQ |
| DI1 | `diModeBit1` | Mode command bit 1 |
| DI2 | `diModeBit2` | Mode command bit 2 (MSB) |
| DI3 | `diMotionEnable` | Motion enable from master |
| DI4 | `diLimitRetract` | Retracted limit switch (PNP NC: TRUE=clear, FALSE=triggered) |
| DI5 | `diLimitHome` | Homing reference limit switch (PNP NC) |
| DI6 | `diFaultReset` | Fault reset command from master |
| DI7 | Reserved | Future use |

### Digital Outputs (8)
| Pin | Signal | Normal Mode | Fault Mode (doFaultActive=HIGH) |
|-----|--------|-------------|--------------------------------|
| DO0 | `doModeConfBit0` | Mode confirm bit 0 | Fault code bit 0 |
| DO1 | `doModeConfBit1` | Mode confirm bit 1 | Fault code bit 1 |
| DO2 | `doModeConfBit2` | Mode confirm bit 2 | Fault code bit 2 |
| DO3 | `doBrakeDisengage` | Brake release (HIGH = disengage) | - |
| DO4 | `doPerformanceStatus` | Mode-dependent status | - |
| DO5 | `doFaultActive` | LOW | HIGH (enables fault code output) |
| DO6 | `doInMotion` | Axis in motion | - |
| DO7 | `doHomingComplete` | Homing complete | - |

### Fault Codes (3-bit, output on DO0-DO2 when doFaultActive=HIGH)
| Code | Fault |
|------|-------|
| 000 | No fault / cleared |
| 001 | Handshake timeout/mismatch |
| 010 | Drive fault |
| 011 | Position limit exceeded |
| 100 | Homing requirement not met |
| 101 | Piston exit prevention triggered |
| 110 | Limit switch fault |
| 111 | Encoder/absolute position fault |

### Analog I/O
| Pin | Signal | Range | Description |
|-----|--------|-------|-------------|
| AI0 | `aiReference` | -10V to +10V | Mode-dependent reference |
| AO0 | `aoPositionFeedback` | -10V to +10V | Position feedback (two-stage scaling) |

## State Machine Architecture

### Complete State Flow
```
                         POWER ON
                             |
                             v
                        [ST_INIT]
                             |
                             v
                   [ST_ENCODER_CHECK]
                        /        \
                   (Valid)    (Invalid)
                      |           |
                      v           v
                 [ST_IDLE] <-- [ST_REQUIRE_ABS_HOME]
                 (Drive OFF,        |
                  Brake ON,         | (flagAbsHomeRequired = TRUE)
                  flagEOTHomeReq    |
                  = TRUE)           |
                      |             |
                      +------+------+
                             |
          Mode Command != 000|
          + diMotionEnable   |
                             v
            +----------------+----------------+
            |                                 |
      (Mode 110,111,101)            (Mode 001-100)
      Homing Modes                  Operational Modes
            |                                 |
            |                    Check: flagAbsHomeReq OR flagEOTHomeReq?
            |                           /              \
            |                       (YES)             (NO)
            |                         |                |
            |                         v                |
            |                   [ST_FAULT]             |
            |                   (FAULT_HOMING_REQ)     |
            |                         |                |
            |                    (Reset)               |
            |                         |                |
            |                         v                |
            +-------------------------+----------------+
                             |
                             v
                    [ST_DRIVE_ENABLE]
                             |
                             v
                    [ST_BRAKE_RELEASE]
                             |
         +-------+-------+---+---+-------+-------+-------+
         |       |       |       |       |       |       |
         v       v       v       v       v       v       v
       [001]   [010]   [011]   [100]   [101]   [110]   [111]
       BRAKE   POS     VEL     TRQ     GO      HOME    HOME
       HOLD    CTRL    CTRL    CTRL    HOME    LIM     EOT
         |       |       |       |       |       |       |
         |       +---+---+---+---+       |       +---+---+
         |               |               |           |
         |    diMotionEnable = LOW       |      [Homing
         |               |               |       Complete]
         |               v               |           |
         |       [ST_HOLD_POSITION] <----+           |
         |       (MC_Halt, brake                     |
         |        disengaged, wait                   |
         |        for next mode                      |
         |        handshake)                         |
         |               |                           |
         |        Handshake for                      |
         |        new mode?                          |
         |          /    \                           |
         |     (YES)    (TIMEOUT)                    |
         |       |          |                        |
         |       v          v                        |
         |   [New Mode]  [ST_FAULT]                  |
         |       |      (HANDSHAKE)                  |
         |       |          |                        |
         +-------+----------+                        |
                 |                                   |
                 |     Clear flags on homing:        |
                 |     - flagAbsHomeReq (110)        |
                 |     - flagEOTHomeReq (111)        |
                 |                 |                 |
                 +<----------------+-----------------+
                 |
            [ST_FAULT]
                 |
        (Fault Recovery
         with handshake)
                 |
                 v
        [ST_BRAKE_ENGAGE]
                 |
                 v
        [ST_DRIVE_DISABLE]
                 |
                 v
            [ST_IDLE]


INTER-MODE TRANSITIONS:
- When diMotionEnable goes LOW in any operational mode → ST_HOLD_POSITION
- ST_HOLD_POSITION: MC_Halt (controlled stop), brake stays disengaged
- Wait for handshake for next mode command
- Handshake timeout → ST_FAULT (no direct path to idle)
- Successful handshake → transition to new operational mode
```

### State Enumeration
```
Initialization:
  ST_INIT              - Power-on initialization
  ST_ENCODER_CHECK     - Validate absolute encoder status
  ST_REQUIRE_ABS_HOME  - Set flagAbsHomeRequired, proceed to idle

Idle:
  ST_IDLE              - Drive OFF, brake engaged, awaiting commands

Transition (Activation):
  ST_DRIVE_ENABLE      - Power on drive, wait for ready
  ST_BRAKE_RELEASE     - Release brake, wait for mechanical delay

Operational:
  ST_BRAKE_HOLD        - Mode 001: Drive ON, brake engaged
  ST_POSITION_CTRL     - Mode 010: Position control active
  ST_VELOCITY_CTRL     - Mode 011: Velocity control active
  ST_TORQUE_CTRL       - Mode 100: Torque control active
  ST_GO_HOME           - Mode 101: Moving to home position
  ST_HOME_LIMIT        - Mode 110: Homing to limit switch
  ST_HOME_EOT          - Mode 111: Homing to end-of-travel
  ST_HOME_COMPLETE     - Homing finished, awaiting mode change

Transition (Inter-Mode):
  ST_HOLD_POSITION     - MC_Halt active, brake disengaged, awaiting next mode handshake

Transition (Deactivation):
  ST_BRAKE_ENGAGE      - Engage brake, wait for mechanical delay
  ST_DRIVE_DISABLE     - Power off drive, return to idle

Fault:
  ST_FAULT             - Fault active, output fault code, await reset
  ST_FAULT_RECOVERY    - Processing fault reset (with handshake)
```

### Homing Sub-States
**Mode 110 (Home to Limit Switch):**
```
ST_HOME_LIM_APPROACH  - Move toward limit switch
ST_HOME_LIM_DETECT    - Detect switch trigger
ST_HOME_LIM_BACKOFF   - Back off from switch
ST_HOME_LIM_SETREF    - Set position reference
```

**Mode 111 (Home to EOT):**
```
ST_HOME_EOT_FAST      - Fast jog toward EOT region (extend direction)
ST_HOME_EOT_SLOW      - Slow torque-limited approach
ST_HOME_EOT_DETECT    - Stall/torque threshold detection
ST_HOME_EOT_SETREF    - Set position reference via MC_SetPosition
```

## Safety Features

### 1. Piston Exit Prevention (ALL Motion Modes)
**Applies to**: Position Control, Velocity Control, Torque Control, Go Home

**Hazard**: Piston exiting cylinder when chamber is pressurized

**Implementation**: Hard position limit
- Enforce `posSoftLimitMin` (cylinder entry boundary + margin)
- Halt/prevent motion beyond boundary in ALL modes
- NOT velocity reduction - immediate position limiting

**Detection in Torque Mode** (additional):
- Monitor for: positive commanded torque + negative actual velocity + near boundary
- This indicates pressure overcoming torque (anomalous condition)

### 2. Handshake Protocol
FB_HandshakeManager handles BOTH:
- **Mode entry handshake**: 3-bit mode command/confirmation with timeout
- **Fault reset handshake**: diFaultReset rising edge + diMotionEnable LOW

Timeout on any handshake failure → fault state

### 3. Position Limits
- Software limits enforced in all motion modes
- Hardware limit switches as backup
- Limit switch wiring: PNP NC (fail-safe: wire break = triggered)

### 4. Homing Requirement Enforcement
- Operational modes blocked until homing requirements satisfied
- Attempting to enter blocked mode → Homing Fault
- Master must explicitly command homing to clear requirements

## Position Reference Management

### Coordinate System
- **Single zero reference** established by homing
- **Positive direction**: Toward cylinder EOT (compression direction)
- **Critical landmarks** tracked as variables:
  - `posLimitRetract` - Position of retract limit switch
  - `posLimitHome` - Position of homing reference switch (= home position)
  - `posCylinderEntry` - Position where piston enters cylinder
  - `posCylinderEOT` - End-of-travel inside cylinder
  - `posSoftLimitMin` - Software position limit (retract/exit prevention)
  - `posSoftLimitMax` - Software position limit (extend)

### Homing Flags
```
flagAbsHomeRequired  : BOOL  - Set if encoder invalid, cleared by Mode 110
flagEOTHomeRequired  : BOOL  - Set on every power-up, cleared by Mode 111
```

## Deliverables and Scope

### LLM-Generated (IEC ST Source Files)
```
/src
  /GVL
    GVL_IO.st             - I/O variable declarations with addresses
    GVL_Config.st         - Configurable parameters
    GVL_System.st         - Runtime state variables
    GVL_Position.st       - Position landmarks and references
  /DUT
    E_SystemState.st      - State enumeration
    E_OperatingMode.st    - Mode enumeration
    E_FaultCode.st        - Fault code enumeration
    ST_AxisLimits.st      - Limit structure
    ST_HomingParams.st    - Homing parameters
  /FB
    FB_ModeDecoder.st         - 3-bit mode decoding
    FB_HandshakeManager.st    - Mode AND fault reset handshake
    FB_FaultCodeOutput.st     - Fault code output logic
    FB_AnalogProcessor.st     - Analog I/O scaling
    FB_PositionMapping.st     - Two-stage position output
    FB_SafetyMonitor.st       - Limit/fault detection
    FB_PistonExitGuard.st     - Piston exit prevention (all modes)
    FB_HomingSequence.st      - Homing state machine (110, 111)
    FB_GoHome.st              - Go Home logic (101)
    FB_EncoderManager.st      - Absolute encoder validation
  /PRG
    PRG_Main.st           - Main program with state machine
```

### Manual Configuration (MotionWorksIEC GUI)
1. **Hardware Configuration**: MP2600iec, SGD7S connection
2. **Axis Configuration**: Gear ratio 70:1, lead 5.08mm, units mm
3. **I/O Address Mapping**: Digital/Analog addresses
4. **Drive Parameters**: Torque/velocity limits, brake settings

### Documentation to Generate
- Hardware configuration guide
- I/O wiring reference
- State machine diagram
- Commissioning checklist

## Implementation Phases

### Phase 1: Project Foundation
**Manual (User)**:
1. Create MotionWorksIEC Express project
2. Configure hardware (MP2600iec, SGD7S)
3. Configure axis (gear ratio, units)

**LLM**:
4. Generate GVL files (IO, Config, System, Position)
5. Generate data type enumerations and structures
6. Generate hardware configuration documentation

### Phase 2: Core Infrastructure
**LLM**:
1. Implement FB_ModeDecoder
2. Implement FB_HandshakeManager (mode entry AND fault reset)
3. Implement FB_FaultCodeOutput
4. Implement FB_AnalogProcessor
5. Implement FB_PositionMapping
6. Create PRG_Main skeleton with state machine

### Phase 3: State Machine
**LLM**:
1. Implement initialization states (INIT, ENCODER_CHECK, REQUIRE_ABS_HOME)
2. Implement transition states (DRIVE_ENABLE, BRAKE_RELEASE, BRAKE_ENGAGE, DRIVE_DISABLE)
3. Implement ST_IDLE with homing requirement checking
4. Implement ST_BRAKE_HOLD
5. Implement operational modes (POSITION, VELOCITY, TORQUE)
6. Implement ST_HOLD_POSITION (MC_Halt, wait for next mode handshake)
7. Implement inter-mode transition logic via ST_HOLD_POSITION

### Phase 4: Homing
**LLM**:
1. Implement FB_EncoderManager (absolute encoder validation)
2. Implement FB_HomingSequence framework
3. Implement limit switch homing (Mode 110)
4. Implement EOT homing (Mode 111) - extend direction, torque detection
5. Implement FB_GoHome (Mode 101)
6. Implement ST_HOME_COMPLETE

### Phase 5: Safety
**LLM**:
1. Implement FB_SafetyMonitor
2. Implement FB_PistonExitGuard (all motion modes)
3. Implement ST_FAULT with fault code output
4. Implement ST_FAULT_RECOVERY with handshake
5. Implement homing requirement fault logic

### Phase 6: Integration
**User + LLM Support**:
1. Import generated .st files into MotionWorksIEC
2. Verify handshake protocol with Simulink master
3. Calibrate analog I/O scaling
4. Tune homing parameters
5. Commission and validate

## Key Implementation Notes

### PLCopen/Yaskawa Function Blocks
- `MC_Power` - Enable/disable drive
- `MC_SetPosition` - Set position reference after homing
- `MC_MoveAbsolute` - Position moves (Go Home)
- `Y_DirectControl` - Cyclic position/velocity/torque control
- `MC_Halt` - Controlled stop
- `MC_Stop` - Emergency stop
- `Y_ResetAbsoluteEncoder` - Clear encoder alarms
- `AbsolutePositionManager` - Encoder validity check

### Brake Control Timing
- Spring-engaged, electrically released
- DO3 HIGH = brake disengaged
- Delays required after disengage/engage

### Limit Switch Logic (PNP NC)
- TRUE (HIGH) = switch NOT triggered (normal)
- FALSE (LOW) = switch triggered OR wire broken (fail-safe)

---

## Phase 4-5 Detailed Implementation Plan

### Design Decisions (User-Confirmed)
1. **Homing sub-states**: Use existing E_SystemState sub-states in PRG_Main (ST_HOME_LIM_APPROACH, etc.)
2. **Stall detection**: Velocity + Torque threshold (velocity < 0.5mm/s AND torque >= 90% threshold for 200ms)
3. **Encoder validation**: Full AbsolutePositionManager implementation

---

### Implementation Order

| Order | File | Description |
|-------|------|-------------|
| 1 | FB_EncoderManager.st | Encoder validity using AbsolutePositionManager |
| 2 | FB_SafetyMonitor.st | Centralized safety monitoring |
| 3 | FB_PistonExitGuard.st | Piston exit prevention for all modes |
| 4 | PRG_Main.st | Homing states (110, 111) + safety integration |
| 5 | FB_GoHome.st | Mode 101 handler with requirement checking |

---

### FB_EncoderManager.st (NEW)

**Purpose**: Check absolute encoder validity on startup using Yaskawa AbsolutePositionManager

**Interface**:
```
FUNCTION_BLOCK FB_EncoderManager
VAR_INPUT
    Enable              : BOOL;
    Axis                : AXIS_REF;
    CheckOnStartup      : BOOL;         (* Trigger startup check *)
END_VAR
VAR_OUTPUT
    EncoderValid        : BOOL;         (* Absolute position is valid *)
    BatteryOK           : BOOL;         (* Encoder battery status *)
    MultiTurnValid      : BOOL;         (* Multi-turn data valid *)
    AbsHomeRequired     : BOOL;         (* Homing required due to encoder issue *)
    CheckComplete       : BOOL;         (* Startup check finished *)
    Error               : BOOL;
    ErrorID             : DWORD;
END_VAR
```

**States**: IDLE → CHECK_BATTERY → CHECK_MULTITURN → CHECK_POSITION → COMPLETE

**Integration**: Called from ST_ENCODER_CHECK, sets flagAbsHomeRequired

---

### FB_SafetyMonitor.st (NEW)

**Purpose**: Monitor all safety conditions, generate fault codes

**Interface**:
```
FUNCTION_BLOCK FB_SafetyMonitor
VAR_INPUT
    Enable              : BOOL;
    LimitRetractActive  : BOOL;         (* Filtered limit switch *)
    LimitHomeActive     : BOOL;
    ActualPosition      : LREAL;
    SoftLimitMin/Max    : LREAL;
    DriveReady/Fault    : BOOL;
    EncoderValid        : BOOL;
    CurrentState        : E_SystemState;
    ExitGuardTriggered  : BOOL;
END_VAR
VAR_OUTPUT
    FaultDetected       : BOOL;
    FaultCode           : E_FaultCode;
    FaultLimitSwitch    : BOOL;
    FaultPosition       : BOOL;
    FaultDrive          : BOOL;
    FaultEncoder        : BOOL;
    FaultPistonExit     : BOOL;
    WarnNearLimitMin/Max: BOOL;
END_VAR
```

**Fault Priority**: Drive > Encoder > LimitSwitch > Position > PistonExit

**Integration**: Called every scan after state machine, triggers ST_FAULT if fault detected

---

### FB_PistonExitGuard.st (NEW)

**Purpose**: Prevent piston exiting cylinder in all motion modes

**Interface**:
```
FUNCTION_BLOCK FB_PistonExitGuard
VAR_INPUT
    Enable              : BOOL;
    CurrentMode         : E_OperatingMode;
    ActualPosition/Velocity/Torque : LREAL;
    CommandedPosition/Velocity/Torque : LREAL;
    SoftLimitMin        : LREAL;
    GuardMargin         : LREAL;        (* cfgPosExitGuardMargin = 5mm *)
END_VAR
VAR_OUTPUT
    CorrectedPosition/Velocity/Torque : LREAL;
    GuardActive         : BOOL;
    PositionLimited     : BOOL;
    VelocityHalted      : BOOL;
    TorqueAnomalyDetect : BOOL;
    FaultTriggered      : BOOL;
END_VAR
```

**Mode-Specific Logic**:
- **Position/GoHome**: Clamp commanded position to >= SoftLimitMin
- **Velocity**: Halt if in danger zone AND commanding negative velocity
- **Torque**: Detect anomaly (negative torque + negative velocity + near boundary) → Fault

---

### FB_GoHome.st (NEW)

**Purpose**: Handle Mode 101 with homing requirement checking

**Interface**:
```
FUNCTION_BLOCK FB_GoHome
VAR_INPUT
    Enable, Execute     : BOOL;
    Axis                : AXIS_REF;
    AbsHomeRequired     : BOOL;         (* flagAbsHomeRequired *)
    EOTHomeRequired     : BOOL;         (* flagEOTHomeRequired *)
    HomePosition        : LREAL;        (* cfgGoHomePosition = 0mm *)
    HomeVelocity        : LREAL;        (* cfgGoHomeVelocity = 50mm/s *)
END_VAR
VAR_OUTPUT
    Active, Done        : BOOL;
    AtPosition          : BOOL;
    HomingRequired      : BOOL;         (* Need homing first *)
    RedirectToHoming    : BOOL;         (* Caller switch to Mode 110 *)
    InMotion            : BOOL;
END_VAR
```

**Logic**:
- If AbsHomeRequired → RedirectToHoming = TRUE (PRG_Main switches to ST_HOME_LIMIT)
- Otherwise → Execute MC_MoveAbsolute to HomePosition

---

### PRG_Main.st Updates

#### Homing State Implementation (using existing E_SystemState sub-states)

**Mode 110 - Home to Limit Switch**:
```
ST_HOME_LIMIT:        Entry: set flagHomingInProgress, start approach
ST_HOME_LIM_APPROACH: MC_MoveVelocity negative, wait for LimitHomeActive
ST_HOME_LIM_DETECT:   MC_Halt, confirm switch triggered
ST_HOME_LIM_BACKOFF:  MC_MoveVelocity positive slow, wait for NOT LimitHomeActive
ST_HOME_LIM_SETREF:   MC_SetPosition(cfgHomeLimSetPosition), clear flagAbsHomeRequired
→ ST_HOME_COMPLETE
```

**Mode 111 - Home to End-of-Travel**:
```
ST_HOME_EOT:          Entry: set flagHomingInProgress, check flagAbsHomeRequired
ST_HOME_EOT_FAST:     MC_MoveVelocity positive fast, until near expected EOT
ST_HOME_EOT_SLOW:     Y_DirectControl velocity + torque limit, monitor stall
ST_HOME_EOT_DETECT:   Stall = velocity<0.5 AND torque>=90% for 200ms
ST_HOME_EOT_SETREF:   MC_SetPosition(cfgHomeEOTSetPosition), clear flagEOTHomeRequired
→ ST_HOME_COMPLETE
```

**ST_HOME_COMPLETE**: Hold position, wait for mode change via handshake

#### Safety Integration

Add to main loop (after state machine, before output update):
```st
(* Safety monitor - every scan *)
fbSafetyMonitor(...);
IF fbSafetyMonitor.FaultDetected AND NOT InFaultState THEN
    sysActiveFault := fbSafetyMonitor.FaultCode;
    sysCurrentState := ST_FAULT;
END_IF

(* Piston exit guard - in operational states *)
fbPistonExitGuard(...);
sysExitGuardActive := fbPistonExitGuard.GuardActive;
```

#### New FB Instances to Add
```st
fbEncoderManager    : FB_EncoderManager;
fbSafetyMonitor     : FB_SafetyMonitor;
fbPistonExitGuard   : FB_PistonExitGuard;
fbGoHome            : FB_GoHome;
```

---

### Critical Files

| File | Action |
|------|--------|
| `src/FB/FB_EncoderManager.st` | CREATE |
| `src/FB/FB_SafetyMonitor.st` | CREATE |
| `src/FB/FB_PistonExitGuard.st` | CREATE |
| `src/FB/FB_GoHome.st` | CREATE |
| `src/PRG/PRG_Main.st` | MODIFY (homing states, safety integration) |
| `src/GVL/GVL_System.st` | MODIFY (add position landmarks if missing) |

---

### Testing Checklist

**Phase 4 (Homing)**:
- [x] Encoder validity check at startup
- [x] Mode 110: Approach, detect, backoff, setref sequence
- [x] Mode 110: flagAbsHomeRequired cleared
- [x] Mode 111: Fast approach, slow approach, stall detect, setref
- [x] Mode 111: flagEOTHomeRequired cleared
- [x] Mode 101: Redirect to 110 when AbsHomeRequired
- [x] Mode 101: Direct move when requirements met
- [x] Homing abort on motion enable drop

**Phase 5 (Safety)**:
- [x] Drive fault detection
- [x] Encoder fault detection
- [x] Unexpected limit switch fault
- [x] Position limit exceeded fault
- [x] Piston exit guard - position mode clamping
- [x] Piston exit guard - velocity mode halt
- [x] Piston exit guard - torque mode anomaly detection
- [x] Fault reset handshake with code mirroring

---

## Documentation Plan for Master Development

### Priority 1: Essential Documents

#### 1. MasterProtocolGuide.md
**Purpose**: Step-by-step implementation guide for Simulink DTRT master

**Contents**:
- Mode handshake sequence with timing diagrams
- Fault reset handshake with mirroring requirement
- State machine template for master implementation
- Code examples (pseudocode/Simulink block recommendations)

**Key Sections**:
```
1. Overview of Master-Slave Architecture
2. Digital I/O Interface Summary
3. Mode Entry Handshake Protocol
   - Sequence diagram
   - Timing requirements (500ms timeout)
   - Master state machine
4. Fault Detection and Recovery
   - Monitoring doFaultActive (DO5)
   - Reading fault code from DO0-DO2
   - Mirroring fault code to DI0-DI2
   - Asserting diFaultReset (DI6)
5. Motion Control via Analog Interface
   - Position reference scaling
   - Velocity reference scaling
   - Torque reference scaling
6. Recommended Master Architecture
   - Handshake state machine
   - Watchdog timers
   - Fault handling subsystem
```

#### 2. IOReference.md
**Purpose**: Complete signal definitions with electrical specifications

**Contents**:
- Digital input/output pin mapping
- Analog I/O scaling formulas
- Two-stage position mapping math
- Signal timing characteristics
- Debounce requirements

#### 3. FaultCodeReference.md
**Purpose**: Fault diagnosis and recovery procedures

**Contents**:
| Code | Name | Cause | Master Recovery Action |
|------|------|-------|------------------------|
| 000 | FAULT_NONE | No fault | N/A |
| 001 | FAULT_HANDSHAKE | Timeout/mismatch | Retry handshake sequence |
| 010 | FAULT_DRIVE | Servo amplifier | Check drive status, reset |
| 011 | FAULT_POSITION | Limit exceeded | Move away from limit |
| 100 | FAULT_HOMING_REQ | Homing needed | Command Mode 110/111 |
| 101 | FAULT_PISTON_EXIT | Safety triggered | Check pressure, reduce force |
| 110 | FAULT_LIMIT_SWITCH | Unexpected trigger | Check mechanics |
| 111 | FAULT_ENCODER | Position invalid | Require homing |

#### 4. StateTransitionDiagram.md
**Purpose**: Visual state machine reference

**Contents**:
- ASCII/Mermaid diagram of all states
- Valid mode transitions table
- Homing requirement enforcement logic
- Inter-mode transition rules

---

### Priority 2: Detailed Reference Documents

#### 5. HomingSequenceGuide.md
**Purpose**: Detailed homing procedure for master coordination

**Contents**:
- Mode 110 (Limit Switch) step-by-step
- Mode 111 (EOT) step-by-step
- Mode 101 (Go Home) behavior
- Homing requirement logic
- Expected durations and timeouts

#### 6. AnalogScalingReference.md
**Purpose**: Complete analog I/O math reference

**Contents**:
- Two-stage position mapping formulas
- Stage 1: 0-200mm → -10V to +5V
- Stage 2: 200-305mm → +5V to +10V
- Inverse mapping for position command
- Velocity scaling: ±100mm/s → ±10V
- Torque scaling: ±100% → ±10V

#### 7. TimingSpecification.md
**Purpose**: All timing parameters in one place

**Contents**:
- Handshake timeout: 500ms
- Brake engage delay: 200ms
- Brake disengage delay: 100ms
- Drive ready timeout: 5s
- Homing timeout: 30s
- Input debounce times
- Analog filter time constant

---

### Priority 3: Supplementary Documents

#### 8. TroubleshootingGuide.md
**Purpose**: Diagnostic procedures for common issues

#### 9. CommissioningChecklist.md
**Purpose**: Step-by-step first-run validation

---

### Implementation Order

| Order | Document | Est. Size |
|-------|----------|-----------|
| 1 | MasterProtocolGuide.md | ~400 lines |
| 2 | IOReference.md | ~200 lines |
| 3 | FaultCodeReference.md | ~150 lines |
| 4 | StateTransitionDiagram.md | ~200 lines |
| 5 | HomingSequenceGuide.md | ~250 lines |
| 6 | AnalogScalingReference.md | ~150 lines |
| 7 | TimingSpecification.md | ~100 lines |

---

### Target Location

All documentation in: `docs/`

```
/docs
  /master
    MasterProtocolGuide.md      <- Primary master development guide
    IOReference.md              <- I/O signal definitions
    FaultCodeReference.md       <- Fault diagnosis
    StateTransitionDiagram.md   <- State machine visual
  /reference
    HomingSequenceGuide.md      <- Homing details
    AnalogScalingReference.md   <- Analog math
    TimingSpecification.md      <- Timing parameters
  /setup
    HardwareConfiguration.md    <- (already exists)
    CommissioningChecklist.md   <- First-run validation
```
