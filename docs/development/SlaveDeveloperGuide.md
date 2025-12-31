# Slave-Side (PLC) Developer Guide

## Gen3 SOS Compressor Servo Controller
### MP2600iec Structured Text Development

---

## 1. Introduction

This guide is intended for developers who need to understand, maintain, or extend the Structured Text (ST) source code for the MP2600iec servo controller (the "slave"). It assumes a basic understanding of IEC 61131-3 programming languages and PLC concepts.

For information regarding the master controller interface, see the [Master Protocol Guide](./master/MasterProtocolGuide.md).

---

## 2. Project Structure

The MotionWorks IEC project is organized into several key folders, following standard PLC programming practices.

| Folder | Name | Purpose |
|--------|------|---------|
| `src/PRG/` | Programs | Contains the main program entry point, `PRG_Main.st`, which houses the core state machine. |
| `src/FB/` | Function Blocks | Contains reusable modules of encapsulated logic (e.g., `FB_HandshakeManager`, `FB_HomeEOT`). This is where most of the application logic resides. |
| `src/GVL/` | Global Variable Lists | Defines global variables accessible across the entire project. This is used for configuration, I/O, system status, and position data. |
| `src/DUT/` | Data Unit Types | Contains definitions for custom data structures (`STRUCT`) and enumerations (`ENUM`), such as `E_SystemState`, `E_OperatingMode`, and `ST_AxisLimits`. |

---

## 3. Coding Conventions

The codebase follows a set of naming conventions to improve readability and maintainability.

### Variable Prefixes

| Prefix | Type | Example | Description |
|--------|------|---------|-------------|
| `b` | `BOOL` | `bEnterState` | A boolean flag. |
| `n` | `INT` / `UINT` | `nValue` | An integer value. |
| `r` | `REAL` / `LREAL` | `rAnalogFilterAlpha` | A floating-point (real) number. |
| `t` | `TIME` / `TON` | `tStateTimer` | A timer instance or time value. |
| `e` | `ENUM` | `eTargetState` | An enumeration value. |
| `st` | `STRUCT` | `stCurrentLimits` | A structure instance. |
| `fb` | Function Block | `fbHandshake` | An instance of a Function Block. |
| `rTrig` | `R_TRIG` | `rTrigFaultReset` | A rising-edge trigger. |
| `fTrig` | `F_TRIG` | `fTrigMotionEnable` | A falling-edge trigger. |

### Global Variable Prefixes

Global variables defined in a `GVL` use a prefix to indicate their scope.

| Prefix | GVL File | Example | Description |
|--------|----------|---------|-------------|
| `cfg` | `GVL_Config.st` | `cfgDebounceTimeMode` | A static configuration parameter used for tuning. |
| `sys` | `GVL_System.st` | `sysCurrentState` | A dynamic system status variable. |
| `pos` | `GVL_Position.st`| `posEOTOffset` | A position-related value, often calculated during homing. |
| `di` / `do` | `GVL_IO.st` | `diModeBit0` | A digital input (`di`) or output (`do`). |
| `ai` / `ao` | `GVL_IO.st` | `aiReference` | An analog input (`ai`) or output (`ao`). |

### Constant Prefixes

| Prefix | Type | Example | Description |
|--------|------|---------|-------------|
| `ST_` | `E_SystemState` ENUM | `ST_IDLE` | A state in the main state machine. |
| `MODE_` | `E_OperatingMode` ENUM | `MODE_POSITION` | An operating mode commanded by the master. |
| `FAULT_` | `E_FaultCode` ENUM | `FAULT_DRIVE` | A system fault code. |

---

## 4. State Machine Deep Dive

The core logic resides in a state machine within `PRG_Main.st`. Understanding this flow is critical.

### Key State Groups

- **Initialization (`ST_INIT`, `ST_ENCODER_CHECK`, `ST_REQUIRE_ABS_HOME`)**: Runs once on startup. Checks encoder validity and sets homing flags.
- **Idle (`ST_IDLE`)**: The default safe state. The drive is disabled, and the brake is engaged. It waits here for a valid handshake from the master.
- **Activation (`ST_DRIVE_ENABLE`, `ST_BRAKE_RELEASE`)**: A transitional sequence to power on the drive and release the brake before entering an operational mode.
- **Operational States (`ST_POSITION_CTRL`, `ST_VELOCITY_CTRL`, etc.)**: The active motion or control states. Each state runs its specific logic (e.g., calling `Y_DirectControl`) and checks for a `diMotionEnable` drop to exit.
- **Homing States (`ST_HOME_LIMIT`, `ST_HOME_EOT`)**: These states execute the encapsulated logic within the `FB_HomeLimit` and `FB_HomeEOT` function blocks.
- **Transition (`ST_HOLD_POSITION`)**: A critical intermediate state entered when `diMotionEnable` drops. It brings the axis to a controlled halt (`MC_Halt`) and waits for a new mode handshake from the master.
- **Deactivation (`ST_BRAKE_ENGAGE`, `ST_DRIVE_DISABLE`)**: A transitional sequence to engage the brake and disable the drive before returning to `ST_IDLE`.
- **Fault Handling (`ST_FAULT`, `ST_FAULT_IDLE`)**:
    - `ST_FAULT`: The active fault state. It performs a controlled `MC_Halt` and waits for a fault reset handshake. It includes logic for a "fast recovery" (to `ST_BRAKE_HOLD`) or a "slow recovery" timeout to `ST_FAULT_IDLE`.
    - `ST_FAULT_IDLE`: The safe, powered-down fault state. The brake is engaged and the drive is off. It waits for a fault reset handshake to return to `ST_IDLE`.

---

## 5. How-To Guides

### How to Add a New Operating Mode

Let's say you want to add `MODE_NEW_FEATURE` with a value of `101` (currently unused).

1.  **Define Enum:** In `src/DUT/E_OperatingMode.st`, add the new mode:
    ```structuredtext
    MODE_NEW_FEATURE := 5,
    ```
    *(Adjust subsequent values if necessary)*.

2.  **Update Conversions:** In `src/DUT/EnumConversions.st`, add the new mode to `ModeToInt` and `IntToMode` functions.

3.  **Update Master Docs:** In `docs/master/MasterProtocolGuide.md` and `docs/master/IOReference.md`, add the new mode to the mode tables so the master developer knows about it.

4.  **Create a State:** In `src/DUT/E_SystemState.st`, add a corresponding state, e.g., `ST_NEW_FEATURE`.

5.  **Add State Logic:** In `PRG_Main.st`, add a new `CASE` statement for `ST_NEW_FEATURE`. Implement the logic for this state.
    ```structuredtext
    ST_NEW_FEATURE:
        IF bEnterState THEN
            sysConfirmedMode := MODE_NEW_FEATURE;
            // ... initialization logic ...
        END_IF

        // ... continuous logic for the new mode ...

        IF fTrigMotionEnable.Q THEN
            sysCurrentState := ST_HOLD_POSITION;
        END_IF
    ```

6.  **Handle Transitions:** In `PRG_Main.st`, update the state transitions to allow entry into your new state.
    *   In `ST_BRAKE_RELEASE`, add: `MODE_NEW_FEATURE: sysCurrentState := ST_NEW_FEATURE;`
    *   In `ST_HOLD_POSITION`, add: `MODE_NEW_FEATURE: sysCurrentState := ST_NEW_FEATURE;`

### How to Add a New Fault Condition

Let's say you want to add `FAULT_NEW_CONDITION` with a value of `5` (currently `FAULT_PISTON_EXIT`).

1.  **Define Enum:** In `src/DUT/E_FaultCode.st`, add the new fault, potentially renumbering others.

2.  **Update Conversions:** In `src/DUT/EnumConversions.st`, add the new fault to `FaultToInt` and `IntToFault`.

3.  **Update Master Docs:** Add the new fault to the tables in `docs/master/FaultCodeReference.md` and other relevant guides.

4.  **Add Detection Logic:** In `src/FB/FB_SafetyMonitor.st`, add the logic that detects the new fault condition.
    ```structuredtext
    (* --------------------------------------------------------------------------
       NEW FAULT MONITORING
       -------------------------------------------------------------------------- *)
    IF (some_new_dangerous_condition) THEN
        FaultNewCondition := TRUE;
    ELSE
        FaultNewCondition := FALSE;
    END_IF
    ```

5.  **Prioritize the Fault:** In the "FAULT PRIORITY AND OUTPUT" section of `FB_SafetyMonitor.st`, add your new fault into the `IF/ELSIF` chain according to its desired priority.
    ```structuredtext
    // Example: Giving it a higher priority than FAULT_POSITION
    ...
    ELSIF FaultLimitSwitch THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_LIMIT_SWITCH;

    ELSIF FaultNewCondition THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_NEW_CONDITION;

    ELSIF FaultPosition THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_POSITION;
    ...
    ```
    The main program (`PRG_Main.st`) will automatically handle the transition to `ST_FAULT` when `fbSafetyMonitor.FaultDetected` becomes true.

---

## 6. Build and Deployment

This section provides a high-level overview of deploying code to the controller.

1.  **Open Project:** Launch Yaskawa MotionWorksIEC and open the project file.
2.  **Connect to Controller:** Ensure you have a network connection to the MP2600iec controller. In the software, connect to the target device.
3.  **Compile Code:**
    *   Right-click on the "Application" in the project tree and select "Clean".
    *   Right-click again and select "Make". This compiles the ST code into a binary format for the controller.
    *   Check the "Messages" window for any compilation errors and resolve them.
4.  **Download to Controller:**
    *   Once compilation is successful, go to "Online" -> "Login". This will connect you to the controller for programming.
    *   The software will prompt you if the code on the PC is different from the code on the controller. Select "Yes" to download the new program.
5.  **Run Program:**
    *   After the download is complete, go to "Online" -> "Run". This will start the execution of the `PRG_Main` program on the controller.
    *   You can now enter "Debug" mode to monitor variable values in real-time.
6.  **Logout:** When finished, go to "Online" -> "Logout" to disconnect from the controller. The program will continue to run.
