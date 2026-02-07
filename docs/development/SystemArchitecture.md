# System Architecture

## Gen3 SOS Compressor Servo Controller
### Software Architecture and Data Flow

---

## 1. Introduction

This document provides a high-level overview of the slave controller's software architecture. It describes the main functional blocks, their interactions, and the flow of data through the system during key operations.

---

## 2. Software Block Diagram

The main program, `PRG_Main`, is structured as a series of processing stages that execute on every 1ms scan cycle.

```
+-----------------------------------------------------------------------------+
|                               PRG_Main Scan Cycle                           |
+-----------------------------------------------------------------------------+
|                                                                             |
|  [Raw Inputs] ----->+-----------------+      +-----------------+             |
|   (di*, ai*)        |  Digital Input  |----->|  FB_ModeDecoder |-----+       |
|                     |     Filters     |      +-----------------+     |       |
|                     +-----------------+                            |       |
|                                                                    |       |
|                     +-----------------+      +-----------------+     |       |
|                     |  Analog Input   |----->| FB_AnalogProc   |-----+       |
|                     |     Filter      |      +-----------------+     |       |
|                     +-----------------+                            |       |
|                                                                    |       |
|  [Drive Feedback] ->+-----------------+                            |       |
|   (Position, Vel)   | MC_ReadActual*  |----------------------------+       |
|                     +-----------------+                            |       |
|                                                                    |       |
|  +-----------------------------------------------------------------+ V     |
|  |                            CORE LOGIC                           |       |
|  |                                                                 |       |
|  |  +-----------------+     +-----------------+     +-----------+  |       |
|  |  | FB_HandshakeMngr|<-+->|                 |<---+|  Homing   |  |       |
|  |  +-----------------+   | | IF-ELSIF States |    |   FBs     |  |       |
|  |                        +>| (State Machine) |<---+ (Go/Lim/EOT)|  |       |
|  |  +-----------------+   | |                 |    +-----------+  |       |
|  |  | Y_DirectControl |<--+ +-----------------+                   |       |
|  |  +-----------------+                                           |       |
|  |                                                                 |       |
|  +-----------------------------------------------------------------+       |
|                                     |                                      |
|  +----------------------------------+----------------------------------+   |
|  V                                                                    V   |
| +--------------------+                                     +-----------------+  |
| | FB_SafetyMonitor   |                                     |  FB_OutputMux   |  |
| +--------------------+                                     +-----------------+  |
|           |                                                          |       |
|           |                                                          |       |
|           +-----> [To State Machine for FAULT transition]             |       |
|                                                                      |       |
|                     +-----------------+      +-----------------+     |       |
|                     | FB_PositionOut  |<-----| Actual Position |     |       |
|                     +-----------------+      +-----------------+     |       |
|                           |                                          |       |
|  [Raw Outputs] <----------+------------------------------------------+       |
|   (do*, ao*)                                                                |
|                                                                             |
+-----------------------------------------------------------------------------+

```

### Component Descriptions

-   **Input Filters:** A layer of digital (`FB_DigitalInputFilter`) and analog (`FB_AnalogInputFilter`) filters that debounce signals and remove noise from the raw I/O, providing stable values for the rest of the program.
-   **Decoders/Processors:** Function blocks that convert raw, filtered inputs into meaningful data (e.g., `FB_ModeDecoder` converts three booleans into an `E_OperatingMode`).
-   **Core Logic (State Machine):** The `CASE` statement in `PRG_Main` that orchestrates all system behavior based on the current state (`G_sysCurrentState`).
-   **Control Blocks (`Y_DirectControl`, `MC_*`):** PLCopen (`MC_`) and Yaskawa-specific (`Y_`) function blocks that provide the low-level interface to the servo drive for commanding motion.
-   **Homing FBs:** Encapsulated logic for the complex homing sequences.
-   **Safety Monitor:** A watchdog (`FB_SafetyMonitor`) that runs every cycle to check for fault conditions.
-   **Output Processors:** Function blocks (`FB_OutputMux`, `FB_PositionOutput`) that format system data into the correct signals for the physical outputs.

---

## 3. Data Flow Analysis

This section describes the path of data during two primary scenarios.

### Scenario 1: Position Command Input

This flow describes how a voltage from the master controller becomes motion.

1.  **Analog Input:** The master's analog voltage command arrives at `G_aiReference`.
2.  **Filtering:** `FB_AnalogInputFilter` processes the raw signal, applying a median filter to reject outliers and a low-pass filter to smooth the signal. The output is `nFilteredAnalog`.
3.  **Scaling:** `FB_AnalogProc` takes `nFilteredAnalog` and, based on the `CurrentMode` being `MODE_POSITION`, scales it using the two-stage mapping parameters into a physical position setpoint in millimeters. The result is stored in `G_sysCommandedPosition`.
4.  **State Machine Execution:** The `ST_POSITION_CTRL` state is active. It takes the `G_sysCommandedPosition`.
5.  **Safety Check:** The `G_sysCommandedPosition` is clamped within the software limits (`stCurrentLimits.PosMin`, `stCurrentLimits.PosMax`).
6.  **Drive Command:** The final, safe position setpoint is passed to the `fbDirectControl.Position` input. `Y_DirectControl` translates this into the proprietary commands for the Yaskawa drive, and the motor moves.

### Scenario 2: Position Feedback Output

This flow describes how the motor's physical position is reported back to the master.

1.  **Encoder Reading:** The Yaskawa drive continuously tracks the motor's absolute encoder position.
2.  **PLCopen Feedback:** The `fbReadActualPos` (`MC_ReadActualPosition`) function block is called every cycle. It communicates with the drive and retrieves the current axis position, placing it in `G_sysActualPosition`.
3.  **Offset Application:** The `fbPosOutput` (`FB_PositionOutput`) function block takes the `G_sysActualPosition` and adds the `G_posEOTOffset` (calculated during EOT homing). This crucial step translates the position from the slave's coordinate system (where 0 is the home limit switch) to the master's coordinate system (where the end-of-travel is a defined value like 300mm).
4.  **Analog Scaling:** `fbPosOutput` then applies the reverse two-stage mapping to the master--referenced position, converting the millimeter value into a raw integer suitable for the analog output DAC.
5.  **Analog Output:** The final raw integer value is sent to `G_aoPositionOutput`, which generates the -10V to +10V signal for the master controller to read.

---

## 4. Key Function Block Roles (`PRG_Main`)

The main program orchestrates a number of critical function blocks.

| Instance | FB Type | Role in Architecture |
|---|---|---|
| `fbFilter*` | `FB_Digital/AnalogInputFilter` | **Input Conditioning:** Provide clean, stable signals to the core logic. |
| `fbModeDecoder`| `FB_ModeDecoder` | **Command Interpretation:** Converts the master's 3-bit command into a meaningful `E_OperatingMode` enum. |
| `fbHandshake`| `FB_HandshakeManager`| **Synchronization:** Manages the entire mode change and fault reset protocol with the master. Essential for system stability. |
| `fbAnalogProc`| `FB_AnalogProcessor`| **Command Scaling:** Translates the filtered analog input voltage into a physical setpoint (position, velocity, or torque). |
| `fbReadActual*`| `MC_ReadActual*` | **Feedback:** The primary source of feedback from the drive, providing real-time position, velocity, and torque. |
| `fbPower` | `MC_Power` | **Drive Control:** Enables and disables power to the servo drive. |
| `fbStop` | `MC_Stop` | **Safe Stop:** Used to execute a controlled stop when changing modes or entering a fault state. |
| `fbDirectControl`| `Y_DirectControl`| **Motion Command:** The primary interface for sending real-time position, velocity, or torque commands to the drive during operational states. |
| `fbHome*` | `FB_HomeLimit`, `FB_HomeEOT` | **Homing Logic:** Encapsulates the complex, multi-step homing sequences. |
| `fbSafetyMonitor`| `FB_SafetyMonitor` | **System Watchdog:** The central authority for detecting unsafe conditions and triggering a fault. |
| `fbOutputMux` | `FB_OutputMux` | **Output Routing:** Switches the meaning of the 3-bit digital outputs between "Mode Confirmation" and "Fault Code". |
| `fbPosOutput` | `FB_PositionOutput` | **Feedback Scaling:** Translates the internal axis position into the correct analog voltage for the master, applying the EOT offset and two-stage mapping. |
