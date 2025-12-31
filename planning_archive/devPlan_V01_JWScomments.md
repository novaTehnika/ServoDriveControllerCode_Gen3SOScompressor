# Implementation Plan: MP2600iec Servo Controller for Gen3 SOS Compressor

## Overview
Develop IEC 61131-3 Structured Text code for a Yaskawa MP2600iec servo controller operating as a slave to a Simulink Desktop Real Time master. The application controls a linear actuator for a medical device that produces oxygen micro-bubble saline solutions.

## Hardware Summary
- **Controller**: Yaskawa MP2600iec (MotionWorksIEC Express)
- **Drive**: Yaskawa Sigma-7 (SGD7S2R8FE0A000300)
- **Motor**: 400W Sigma-7 with brake and absolute encoder (SGM7J-04A6A6C)
- **Actuator**: Tolomatic RSA64 (~305mm usable stroke)
- **Gearing**: 70:1 motor-to-leadscrew, 5.08mm per leadscrew revolution
- **I/O**: 8 DI, 8 DO, 1 AI (-10V to +10V), 1 AO
- **Master Interface**: NI PCI 6251 DAQ via analog/digital signals
[[COMMENT TO LLM]]
Note also the limit switches. They are Tolomatic part # 8100-9092 and are "Solid State - Normally Closed - PNP". (this is correctly noted below but I would like it to be documented clearly.)
[[/COMMENT TO LLM]]

## Operating Modes (3-bit encoding)
| Code | Mode | Description |
|------|------|-------------|
| 000 | Idle | No motion, brake engaged |
| 001 | Brake Hold | Brake engaged, holding position |
| 010 | Position Control | Analog input as position reference |
| 011 | Velocity Control | Analog input as velocity reference |
| 100 | Torque Control | Analog input as torque reference |
| 101 | Reserved | Not used |
| 110 | Home to Limit Switch | Home using physical limit switch |
| 111 | Home to End-of-Travel | Home via torque-limited stall detection (extend direction) |

[[COMMENT TO LLM]]
Idle state should be drive power off as well as brake engaged (i.e. not disengaged). 
[[/COMMENT TO LLM]]

## Digital I/O Mapping

### Digital Inputs (8)
| Pin | Signal | Description |
|-----|--------|-------------|
| DI0 | `diModeBit0` | Mode command bit 0 (LSB) from NI DAQ |
| DI1 | `diModeBit1` | Mode command bit 1 |
| DI2 | `diModeBit2` | Mode command bit 2 (MSB) |
| DI3 | `diMotionEnable` | Motion enable from master |
| DI4 | `diLimitRetract` | Retracted position limit switch (PNP, NC) |
| DI5 | `diLimitHome` | Homing reference limit switch (PNP, NC) |
| DI6 | `diFaultReset` | Fault reset command from master |
| DI7 | Reserved | Future use |

### Digital Outputs (8)
| Pin | Signal | Description |
|-----|--------|-------------|
| DO0 | `doModeConfBit0` | Mode confirmation bit 0 (LSB) to NI DAQ |
| DO1 | `doModeConfBit1` | Mode confirmation bit 1 |
| DO2 | `doModeConfBit2` | Mode confirmation bit 2 (MSB) |
| DO3 | `doBrakeDisengage` | Brake release (HIGH = disengage) |
| DO4 | `doPerformanceStatus` | Mode-dependent status (limited/complete) |
| DO5 | `doFaultActive` | Fault indicator (HIGH = fault) |
| DO6 | `doInMotion` | Axis in motion indicator |
| DO7 | `doHomingComplete` | Homing sequence complete |

### Analog I/O
| Pin | Signal | Range | Description |
|-----|--------|-------|-------------|
| AI0 | `aiReference` | -10V to +10V | Mode-dependent reference |
| AO0 | `aoPositionFeedback` | -10V to +10V | Position feedback (two-stage scaling) |

## State Machine States

### Primary States
- `ST_INIT` - Power-on initialization
- `ST_WAIT_DRIVE_READY` - Wait for servo drive ready
- `ST_IDLE` - Mode 000: No motion
[[COMMENT TO LLM]]
I think its worth reconsidering the states ST_WAIT_DRIVE_READY, ST_IDLE, and ST_BRAKE_HOLD. Perhaps ST_BRAKE_HOLD can be an a transition state between idle and the others.
[[/COMMENT TO LLM]]
- `ST_BRAKE_HOLD` - Mode 001: Brake engaged
- `ST_POSITION_CTRL` - Mode 010: Position control active
- `ST_VELOCITY_CTRL` - Mode 011: Velocity control active
- `ST_TORQUE_CTRL` - Mode 100: Torque control active
- `ST_HOME_LIMIT` - Mode 110: Homing to limit switch
- `ST_HOME_EOT` - Mode 111: Homing to end-of-travel
- `ST_HOME_COMPLETE` - Homing finished, awaiting mode change

### Transition States
- `ST_TRANS_TO_*` - One per operating mode for handshake validation

### Homing Sub-States (Mode 111)
- `ST_HOME_EOT_FAST` - Fast jog toward end-of-travel region
- `ST_HOME_EOT_SLOW` - Slow torque-limited approach
- `ST_HOME_EOT_DETECT` - Stall/torque threshold detection
- `ST_HOME_EOT_SETREF` - Set position reference
- 
[[COMMENT TO LLM]]
On homing and setting position reference:
1. We need to track multiple position references. The end of travel will be variable with respect to the position of the actuator in its travel and there by to the limit switches.
2. We should also consider how we will deal with setting zero for the absolute encoder. The drive tracks this and has a battery to maintain the number of turns but if that dies or is disconnected we might need to reset it manually. Is there a means of resetting the absolute position tracked by the drive by writing parameters from the controller? We should think through this.
[[/COMMENT TO LLM]]

### Fault States
- `ST_FAULT` - General fault
- `ST_FAULT_HANDSHAKE` - Handshake timeout/mismatch
- `ST_FAULT_LIMIT` - Limit switch fault
- `ST_FAULT_DRIVE` - Drive fault
- `ST_FAULT_POSITION` - Position limit exceeded
- `ST_FAULT_TORQUE` - Unsafe torque condition
- `ST_FAULT_RECOVERY` - Fault reset in progress

[[COMMENT TO LLM]]
I think we need a way to communicate the faults. We should repurpose the three DO pins currently designated for mode confirmation in combination with raising the doFaultActive pin.
Perhaps we should have a handshake for resetting the faults as well?
[[/COMMENT TO LLM]]

## Key Safety Features
1. **Handshake Protocol**: 3-bit mode command/confirmation with timeout (enter fault on failure)
2. **Piston Exit Prevention**: Limit torque when position near cylinder exit and torque is positive
3. **Mode-Dependent Limits**: Configurable position/velocity/torque limits per mode
4. **Soft Position Limits**: Software-enforced in addition to hardware STO
5. **Velocity Limiting in Torque Mode**: Restrict velocity based on expected discharge rates

[[COMMENT TO LLM]]
Item 2: the goal is to prevent the piston from exiting the cylinder when the chamber is under pressure. The servo should not allow *motion* out of the cylinder not *limit torque*. Torque is being sensed to indicate the hazardous condition. Torque should not be limited in any way. Limiting the torque could actually lead to piston being driven out of the cylinder, which is what we want to avoid.
Item 5: On torque mode velocity limiting - we might also want to limit negative velocity as well. Nominal conditions are that positive motion is required to compress the mixture and drive it out. Negative motion to maintain a specified torque is anomalous. The only condition in which I can imagine this would happen would be due to combustion (we are creating a high oxygen atmosphere and are compressing it rapidly) or if the device was being used for something other than its intended function.
[[/COMMENT TO LLM]]

## Files to Create

### Project Structure
```
/Project
  /GVL
    GVL_IO.st           - I/O variable mappings
    GVL_Config.st       - Configurable parameters
    GVL_System.st       - System state variables
  /DUT
    E_SystemState.st    - State enumeration
    E_OperatingMode.st  - Mode enumeration
    E_FaultCode.st      - Fault code enumeration
    ST_AxisLimits.st    - Limit structure
    ST_HomingParams.st  - Homing parameters structure
  /FB
    FB_ModeDecoder.st       - 3-bit mode decoding
    FB_HandshakeManager.st  - Master-slave handshake
    FB_AnalogProcessor.st   - Analog I/O scaling
    FB_SafetyMonitor.st     - Limit/fault detection
    FB_HomingSequence.st    - Homing state machine
    FB_PositionFeedback.st  - Two-stage position output
  /PRG
    PRG_Main.st         - Main program with state machine
```
[[COMMENT TO LLM]]
On "FB_PositionFeedback.st" naming. I think "FB_PositionMapping.st" may be a clearer.
[[/COMMENT TO LLM]]

## Implementation Phases

### Phase 1: Project Setup
1. Create MotionWorksIEC Express project
2. Configure hardware (MP2600iec, SGD7S connection)
3. Configure axis (gear ratio: 70:1, lead: 5.08mm, units: mm)
4. Create GVL_IO with I/O address mappings
5. Create data type enumerations and structures

[[COMMENT TO LLM]]
Items 1, 2, and 3 will be performed outside the scope of the LLM tasks. This should be made clear. However documentation would be a great LLM task.
Would items 4 and 5 be outside scope as well? Where are these instantiated? If in MotionWorksIEC Express than the output should just be documentation.
[[/COMMENT TO LLM]]

### Phase 2: Core Infrastructure
1. Implement GVL_Config with all configurable parameters
2. Implement GVL_System for runtime state
3. Implement FB_ModeDecoder
4. Implement FB_HandshakeManager
5. Implement FB_AnalogProcessor (including two-stage position feedback)
6. Create PRG_Main skeleton with state machine CASE structure

### Phase 3: Operating Modes
1. Implement ST_INIT and ST_WAIT_DRIVE_READY
2. Implement ST_IDLE with mode transition logic
3. Implement ST_BRAKE_HOLD
4. Implement ST_POSITION_CTRL using Y_DirectControl
5. Implement ST_VELOCITY_CTRL
6. Implement ST_TORQUE_CTRL with piston exit prevention
[[COMMENT TO LLM]]
Note comments above on reconsidering none operational states.
[[/COMMENT TO LLM]]

### Phase 4: Homing
1. Implement FB_HomingSequence framework
2. Implement limit switch homing (Mode 110)
3. Implement EOT homing (Mode 111) - extend direction, torque detection
4. Implement ST_HOME_COMPLETE

### Phase 5: Safety and Faults
1. Implement FB_SafetyMonitor
2. Implement all ST_FAULT_* states
3. Implement ST_FAULT_RECOVERY with MC_Reset
4. Test fault injection and recovery

### Phase 6: Integration
1. Verify handshake protocol with Simulink master
2. Calibrate analog I/O scaling
3. Tune homing parameters
4. Full system validation

## Key Implementation Notes

### Y_DirectControl Usage
- Mode 1: Cyclic position setpoint
- Mode 2: Cyclic velocity setpoint
- Mode 3: Cyclic torque setpoint
- Updates every scan cycle (recommend 2-4ms task cycle)

### Analog Scaling
- Raw INT: -32768 to +32767 = -10V to +10V
- Position feedback uses two-stage mapping for higher resolution in-cylinder

### Units
- Position: millimeters (mm)
- Velocity: mm/s
- Torque: % of rated (100 = 100%, up to 800% peak)

### Brake Control
- Spring-engaged, electrically released
- DO3 HIGH = brake disengaged (motion allowed)
- Include delay after disengage before motion, delay after engage before drive disable

### Limit Switches
- PNP sourcing, normally closed (NC)
- TRUE = not at limit, FALSE = at limit (fail-safe wiring)

### Homing Direction (Mode 111)
- Move in **extend** (positive) direction toward piston end-of-travel
- Fast jog to approach region, then slow torque-limited detection