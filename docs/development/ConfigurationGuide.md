# Configuration and Tuning Guide

## Gen3 SOS Compressor ServoController
### Configuration Parameter Reference

---

## 1. Introduction

This document provides a detailed reference for all configurable parameters documented in `src/GVL/GlobalVariables_Reference.st` and defined in the MWiec GUI. These variables allow the system's behavior to be tuned and adapted without changing the core application logic.

**Warning:** Modifying these parameters can have a significant impact on machine performance and safety. Changes should be made carefully and tested thoroughly.

**Units:**
-   **Position:** millimeters (mm)
-   **Velocity:** millimeters per second (mm/s)
-   **Torque:** Percent of motor's rated torque (%)
-   **Time:** Milliseconds (ms) or Seconds (s)

---

## 2. Timing Parameters

These parameters control the duration of timeouts and delays within the state machine.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgHandshakeTimeout` | T#500MS | ms | Max time for the master-slave mode handshake. **Tuning:** Increase if the master controller is slow and causes nuisance handshake faults. Decrease for faster fault detection of an unresponsive master. |
| `cfgModeTransitionDelay` | T#100MS | ms | A small delay used in some state transitions to ensure stability. Generally should not be changed. |
| `cfgBrakeEngageDelay` | T#200MS | ms | Time to wait after commanding the brake to engage before proceeding. Should be slightly longer than the mechanical brake's specified engagement time. |
| `cfgBrakeDisengageDelay`| T#100MS | ms | Time to wait after commanding the brake to release before proceeding. Should be slightly longer than the mechanical brake's specified release time. |
| `cfgHomingTimeout` | T#30S | s | Maximum time allowed for an entire homing sequence (Mode 110 or 111) to complete. **Tuning:** Increase if the travel distance is long or homing velocities are very slow. |
| `cfgStallDetectTime` | T#200MS | ms | The duration a stall condition (high torque, low velocity) must be met continuously during EOT homing to be considered a valid hard stop. **Tuning:** Increase to prevent false stall detections from friction. Decrease for faster homing, but with higher risk of false detection. |
| `cfgDriveReadyTimeout` | T#5S | s | Max time to wait for the servo drive to report "Ready" after being enabled. Should not need adjustment unless the drive has an unusually long power-on sequence. |
| `cfgFaultIdleTimeout` | T#30S | s | When a fault occurs, the system enters a controlled stop. If the master does not acknowledge/reset the fault within this time, the slave will proceed to a safe-idle state (drive off, brake on). |

---

## 3. Position and Velocity Limits

These define the safe operating envelope of the machine.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgPosSoftLimitMax` | 305.0 | mm | Software-enforced maximum extension position. A `FAULT_POSITION` occurs if exceeded. Should be set within the `cfgUsableStroke`. |
| `cfgPosSoftLimitMin` | 0.0 | mm | Software-enforced minimum retraction position (home position). A `FAULT_POSITION` occurs if exceeded. |
| `cfgPosExitGuardMargin`| 5.0 | mm | Safety margin for the piston exit guard. The guard becomes active when the position is `cfgPosSoftLimitMin + cfgPosExitGuardMargin`. |
| `cfgVelLimitMax` | 200.0 | mm/s | Absolute maximum velocity allowed in any mode. |
| `cfgVelLimitNormal` | 100.0 | mm/s | The velocity limit used during standard Position Control mode. |
| `cfgInMotionVelThreshold`| 0.1 | mm/s | The velocity above which the `doInMotion` output is considered active. **Tuning:** Increase if motor vibration at rest causes the flag to flicker. |

---

## 4. Torque Limits

These parameters control the force applied by the motor.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgTorqueLimitMax` | 300.0 | % | Absolute maximum torque allowed. This is a primary safety limit to prevent mechanical damage. |
| `cfgTorqueLimitNormal`| 100.0 | % | Torque limit used during standard Position and Velocity control modes. |
| `cfgTorqueHomingLimit`| 60.0 | % | Torque limit applied during the final slow approach of EOT Homing (Mode 111). This should be high enough to overcome friction but low enough to avoid damaging the mechanical hard stop. |

---

## 5. Input Filtering

These parameters control the software filters applied to raw I/O signals to reject noise.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgDebounceTimeMode`| T#50MS | ms | Debounce time for the mode command bits. **Tuning:** Increase if noisy signals cause mode commands to be unstable. Decrease for faster mode changes. |
| `cfgDebounceTimeMotion`| T#20MS | ms | Debounce time for the Motion Enable signal. Should be relatively short for responsiveness. |
| `cfgDebounceTimeFault`| T#20MS | ms | Debounce time for the Fault Reset signal. |
| `cfgDebounceTimeLimitStd`| T#5MS | ms | Debounce time for the Retract limit switch. |
| `cfgDebounceTimeLimitHome`| T#2MS | ms | Debounce time for the Home limit switch. This is shorter to increase the precision and repeatability of the Home to Limit sequence. |
| `cfgAnalogFilterTimeConst`| T#50MS | ms | Time constant for the IIR low-pass filter on the analog reference input. **Tuning:** Increase for smoother but delayed response. Decrease for faster but potentially noisier response. |
| `cfgAnalogMedianSize`| 3 | count | The number of samples for the median filter on the analog input (valid: 3 or 5). The median filter is excellent at rejecting large, single-sample spikes. |

---

## 6. Homing Parameters

These parameters define the behavior of the three homing modes.

### Mode 110: Home to Limit Switch
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgHomeLimApproachVel`| 50.0 | mm/s | The velocity used to approach the home limit switch. |
| `cfgHomeLimBackoffDist`| 5.0 | mm | The distance the axis moves away from the switch after triggering it, before setting the final home position. |
| `cfgHomeLimSetPosition`| 0.0 | mm | The absolute position value that will be assigned to the axis once the homing is complete. This defines the origin of the coordinate system. |

### Mode 111: Home to End-of-Travel
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgHomeEOTFastVel` | 50.0 | mm/s | The velocity for the initial fast approach toward the hard stop. |
| `cfgHomeEOTSlowVel` | 5.0 | mm/s | The final, slow velocity for the torque-limited contact with the hard stop. |
| `cfgHomeEOTApproachDist`| 20.0 | mm | The distance from the expected EOT position at which the axis will switch from fast to slow velocity. |
| `cfgHomeEOTTorqueThresh`| 50.0 | % | The torque level that must be reached (as a percentage of `cfgTorqueHomingLimit`) to detect a stall. **Tuning:** This is a critical parameter. Set it high enough to avoid false stall from friction, but low enough to reliably detect contact with the hard stop. |
| `cfgHomeEOTSetPosition`| 300.0 | mm | The "ideal" master coordinate position at the EOT. The slave calculates an offset so that the master sees this position value when the slave is at the physical EOT. |

### Mode 101: Go Home
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `cfgGoHomePosition` | 0.0 | mm | The target position for the Go Home command. This should match `cfgHomeLimSetPosition`. |
| `cfgGoHomeVelocity` | 50.0 | mm/s | The velocity used for the move-to-home motion. |

---

## 7. Analog I/O Scaling

These parameters define how analog voltages are mapped to physical units. For more details, see the [IOReference](./master/IOReference.md).

| Parameter | Default Value | Units | Description |
|---|---|---|---|
| `cfgAnalogVelMin` | -100.0 | mm/s | Velocity corresponding to a -10V command. |
| `cfgAnalogVelMax` | 100.0 | mm/s | Velocity corresponding to a +10V command. |
| `cfgAnalogTorqueMin`| -100.0 | % | Torque corresponding to a -10V command. |
| `cfgAnalogTorqueMax`| 100.0 | % | Torque corresponding to a +10V command. |
| `cfgPosMapStage1PosMin` | 0.0 | mm | Start position for the high-resolution mapping region. |
| `cfgPosMapStage1VoltMin`| -10.0 | V | Voltage corresponding to the start position of Stage 1. |
| `cfgPosMapTransitionPos`| 200.0| mm | Position where mapping changes from Stage 1 to Stage 2. |
| `cfgPosMapTransitionVolt`| 5.0 | V | Voltage at the transition point. |
| `cfgPosMapStage2PosMax` | 305.0 | mm | End position for the low-resolution mapping region. |
| `cfgPosMapStage2VoltMax` | 10.0 | V | Voltage corresponding to the end position of Stage 2. |
