# Implementation Plan: MP2600iec Servo Controller for Gen3 SOS Compressor
## Revision 2 - Incorporating User Feedback

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
| 101 | Reserved | Not used |
| 110 | Home to Limit Switch | Home using physical limit switch |
| 111 | Home to End-of-Travel | Home via torque-limited stall detection (extend direction) |

[[COMMENT TO LLM]]
Lets use the 101 code for a Go Home Command. Home will be the location of the homing limit switch. Executing the Go Home command should satisfy the homing requirement if it is other wise unmet (if req is not met and go home is commanded, execute home to limit switch instead the Go Home routine should not cause change to position variable/settings unless the homing req. has not been met).
[[/COMMENT TO LLM]]

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
| 100 | Velocity limit exceeded |
| 101 | Piston exit prevention triggered |
| 110 | Limit switch fault |
| 111 | Encoder/homing fault |

### Analog I/O
| Pin | Signal | Range | Description |
|-----|--------|-------|-------------|
| AI0 | `aiReference` | -10V to +10V | Mode-dependent reference |
| AO0 | `aoPositionFeedback` | -10V to +10V | Position feedback (two-stage scaling) |


## State Machine Architecture

### Revised State Flow
```
                    POWER ON
                        |
                        v
                   [ST_INIT]
                        |
                        v
              [ST_ENCODER_CHECK] -----> (Invalid) ----> [ST_REQUIRE_HOME]
                        |                                      |
                    (Valid)                                    |
                        |                                      |
                        v                                      v
                   [ST_IDLE] <---------------------------------+
                   (Drive OFF,                          (After homing)
                   Brake ON)
                        |
                        | Mode Command (001-111)
                        v
                [ST_DRIVE_ENABLE]  <-- Transition state
                (Power on drive,
                 wait ready)
                        |
                        v
                [ST_BRAKE_RELEASE] <-- Transition state
                (Release brake,
                 wait delay)
                        |
                        +-------+-------+-------+-------+-------+
                        |       |       |       |       |       |
                        v       v       v       v       v       v
                   [010]   [011]   [100]   [110]   [111]   [001]
                   POS     VEL     TRQ     HOME    HOME    BRAKE
                   CTRL    CTRL    CTRL    LIM     EOT     HOLD
                        |       |       |       |       |
                        +-------+-------+-------+-------+
                                        |
                                        | Mode 000 or Fault
                                        v
                              [ST_BRAKE_ENGAGE]
                              (Engage brake,
                               wait delay)
                                        |
                                        v
                              [ST_DRIVE_DISABLE]
                              (Power off drive)
                                        |
                                        v
                                   [ST_IDLE]
```
[[COMMENT TO LLM]]
Note this is not exhaustive. Needs transitions out of and between Operational modes.
[[/COMMENT TO LLM]]

[[COMMENT TO LLM]]
I have some thoughts on implementing faults and homing. We should require homing sequences before allowing normal operational modes. If we try to enter an operational mode (verified by handshake) while not having resolved requirements for homing, we enter the relevant homing fault. We exit the fault (with handshake) but enter it again if the homing requirement hasn't been met. We need the instruction of the master to perform any motion including homing. This pattern ensures the slave can enforce the homing requirements while still requiring instruction from the master to do so.
The requirements are determined by the encoder check shown in the above schematic for absolute home and for all power ups for the EOT homing.
[[/COMMENT TO LLM]]

### State Enumeration
```
Initialization:
  ST_INIT              - Power-on initialization
  ST_ENCODER_CHECK     - Validate absolute encoder status
  ST_REQUIRE_HOME      - Encoder invalid, homing required before operation

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
  ST_HOME_LIMIT        - Mode 110: Homing to limit switch
  ST_HOME_EOT          - Mode 111: Homing to end-of-travel
  ST_HOME_COMPLETE     - Homing finished, awaiting mode change

Transition (Deactivation):
  ST_BRAKE_ENGAGE      - Engage brake, wait for mechanical delay
  ST_DRIVE_DISABLE     - Power off drive, return to idle

Fault:
  ST_FAULT             - Fault active, output fault code, await reset
  ST_FAULT_RECOVERY    - Processing fault reset
```

### Homing Sub-States (Mode 111 - EOT)
```
ST_HOME_EOT_FAST    - Fast jog toward end-of-travel region (extend direction)
ST_HOME_EOT_SLOW    - Slow torque-limited approach
ST_HOME_EOT_DETECT  - Stall/torque threshold detection
ST_HOME_EOT_SETREF  - Set position reference via MC_SetPosition
```

## Position Reference Management

### Coordinate System
- **Single zero reference** established by homing
- **Positive direction**: TBD (recommend: toward cylinder EOT for compression)
- **Critical landmarks** tracked as constants/variables:
  - `posLimitRetract` - Position of retract limit switch
  - `posLimitHome` - Position of homing reference switch
  - `posCylinderEntry` - Position where piston enters cylinder
  - `posCylinderEOT` - End-of-travel inside cylinder
  - `posSoftLimitMin` - Software position limit (retract)
  - `posSoftLimitMax` - Software position limit (extend)

### Absolute Encoder Handling
On startup, check encoder validity using `AbsolutePositionManager`:
1. If valid: proceed to ST_IDLE
2. If invalid (battery failure, etc.):
   - Transition to ST_REQUIRE_HOME
   - Block all operational modes except homing
   - After successful homing, use `MC_SetPosition` to establish reference
   - Clear encoder alarms with `Y_ResetAbsoluteEncoder` if needed


## Safety Features

### 1. Piston Exit Prevention (Torque Mode)
**Hazard**: In torque mode, chamber pressure may overcome commanded torque, pushing piston toward cylinder exit.

**Detection**:
- Position approaching cylinder exit boundary (`pos < posCylinderEntry + margin`)
- Positive commanded torque
- Negative actual velocity (moving toward exit)

**Response**:
- Enforce software position limit (prevent motion beyond boundary)
- NOT emergency stop - use position limiting behavior
- Brake available as backup (capacity = motor rated torque)

[[COMMENT TO LLM]]
We need this for position velocity control modes as well.
[[/COMMENT TO LLM]]

### 2. Velocity Limiting in Torque Mode
- Limit **positive** velocity based on expected discharge rates
- Limit **negative** velocity more strictly (anomalous condition)
- Negative motion while commanding positive torque indicates:
  - Possible combustion event (high O2 atmosphere)
  - System misuse

[[COMMENT TO LLM]]
We might just make use of the Piston Exit Prevention to cover this. If combustion was happening it would actually be safer to allow expansion while also enforcing Piston Exit Prevention.
[[/COMMENT TO LLM]]

### 3. Handshake Protocol
- 3-bit mode command from master (DI0-2)
- 3-bit mode confirmation to master (DO0-2)
- Timeout on handshake failure → fault state
- Fault reset handshake: diFaultReset rising edge + diMotionEnable LOW

### 4. Position Limits
- Software limits enforced in all motion modes
- Hardware limit switches as backup
- Limit switch wiring: PNP NC (fail-safe: wire break = triggered)

## Deliverables and Scope

### LLM-Generated (IEC ST Source Files)
These `.st` files will be importable into MotionWorksIEC:

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
    FB_HandshakeManager.st    - Master-slave handshake
    FB_FaultCodeOutput.st     - Fault code output logic
    FB_AnalogProcessor.st     - Analog I/O scaling
    FB_PositionMapping.st     - Two-stage position output (renamed)
    FB_SafetyMonitor.st       - Limit/fault detection
    FB_PistonExitGuard.st     - Piston exit prevention logic
    FB_HomingSequence.st      - Homing state machine
    FB_EncoderManager.st      - Absolute encoder validation
  /PRG
    PRG_Main.st           - Main program with state machine
```

### Manual Configuration (MotionWorksIEC GUI)
These require manual setup with documentation provided:

1. **Hardware Configuration**
   - MP2600iec controller setup
   - SGD7S drive connection (Mechatrolink)

2. **Axis Configuration**
   - Gear ratio: 70:1
   - Lead screw pitch: 5.08 mm/rev
   - Position units: millimeters
   - Velocity units: mm/s
   - Encoder resolution settings

3. **I/O Address Mapping**
   - Digital I/O addresses (%IX0.0, %QX0.0, etc.)
   - Analog I/O addresses (%IW0, %QW0)

4. **Drive Parameters**
   - Torque limits
   - Velocity limits
   - Brake control settings

### Documentation to Generate
- Hardware configuration guide (step-by-step for MotionWorksIEC)
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
2. Implement FB_HandshakeManager
3. Implement FB_FaultCodeOutput
4. Implement FB_AnalogProcessor
5. Create PRG_Main skeleton with state machine

[[COMMENT TO LLM]]
FB_HandshakeManager should handle both operational mode entry AND fault reset or else a separate handshake manager is needed for fault reset.
Add "Implement FB_PositionMapping" to phase 2.
[[/COMMENT TO LLM]]

### Phase 3: State Machine
**LLM**:
1. Implement initialization states (INIT, ENCODER_CHECK, REQUIRE_HOME)
2. Implement transition states (DRIVE_ENABLE, BRAKE_RELEASE, BRAKE_ENGAGE, DRIVE_DISABLE)
3. Implement ST_IDLE
4. Implement ST_BRAKE_HOLD
5. Implement operational modes (POSITION, VELOCITY, TORQUE)

### Phase 4: Homing
**LLM**:
1. Implement FB_EncoderManager (absolute encoder validation)
2. Implement FB_HomingSequence framework
3. Implement limit switch homing (Mode 110)
4. Implement EOT homing (Mode 111) - extend direction, torque detection
5. Implement ST_HOME_COMPLETE

[[COMMENT TO LLM]]
Add "Implement FB_GoHome" (see comment above on Go Home Command) in phase 4
[[/COMMENT TO LLM]]

### Phase 5: Safety
**LLM**:
1. Implement FB_SafetyMonitor
2. Implement FB_PistonExitGuard
3. Implement ST_FAULT with fault code output
4. Implement ST_FAULT_RECOVERY
5. Add velocity limiting logic for torque mode

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
- `Y_DirectControl` - Cyclic position/velocity/torque control
- `MC_Halt` - Controlled stop
- `MC_Stop` - Emergency stop
- `Y_ResetAbsoluteEncoder` - Clear encoder alarms
- `AbsolutePositionManager` - Encoder validity check

### Brake Control Timing
- Spring-engaged, electrically released
- DO3 HIGH = brake disengaged
- Delays required:
  - After disengage: wait before commanding motion
  - After engage: wait before disabling drive

### Limit Switch Logic (PNP NC)
- TRUE (HIGH) = switch NOT triggered (normal)
- FALSE (LOW) = switch triggered OR wire broken (fail-safe)
