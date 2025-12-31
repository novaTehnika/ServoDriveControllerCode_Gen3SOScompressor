# Fault Code Reference

## Gen3 SOS Compressor Servo Controller
### Fault Diagnosis and Recovery Procedures

---

## 1. Overview

When `doFaultActive` (DO5) is HIGH, the slave is in fault state and DO0-DO2 contain the fault code instead of mode confirmation. The master must handle fault conditions by:

1. Reading the fault code
2. Mirroring the code on DI0-DI2
3. Asserting `diFaultReset` with `diMotionEnable` LOW
4. Waiting for fault to clear

---

## 2. Fault Code Summary

| Binary | Decimal | Name | Severity | Auto-Recoverable |
|--------|---------|------|----------|------------------|
| 000 | 0 | FAULT_NONE | - | - |
| 001 | 1 | FAULT_HANDSHAKE | Medium | Yes |
| 010 | 2 | FAULT_DRIVE | High | Sometimes |
| 011 | 3 | FAULT_POSITION | Medium | Yes |
| 100 | 4 | FAULT_HOMING_REQ | Low | Yes (with homing) |
| 101 | 5 | FAULT_PISTON_EXIT | High | Conditional |
| 110 | 6 | FAULT_LIMIT_SWITCH | High | No (investigation) |
| 111 | 7 | FAULT_ENCODER | High | No (homing required) |

---

## 3. Detailed Fault Descriptions

### FAULT_NONE (000)

**Description**: No fault condition present.

**When Seen**: After successful fault reset, or normal operation.

**Master Action**: None required.

---

### FAULT_HANDSHAKE (001)

**Description**: Mode handshake timeout or mismatch.

**Triggered When**:
- Mode command not confirmed within 500ms timeout
- Mode bits changed during handshake
- Mode confirmation doesn't match command

**Symptoms**:
- Slave stuck in transition state
- Mode bits stable but no confirmation

**Master Recovery**:
```
1. Read fault code (001)
2. Set diMotionEnable = FALSE
3. Mirror code: DI0=1, DI1=0, DI2=0
4. Pulse diFaultReset
5. After clear: retry mode command
```

**Prevention**:
- Ensure mode bits are stable before handshake
- Don't change mode during handshake
- Verify I/O wiring integrity

---

### FAULT_DRIVE (010)

**Description**: Servo amplifier reported a fault condition.

**Triggered When**:
- Overcurrent detected
- Overvoltage/undervoltage
- Motor overheating
- Encoder communication error
- Internal drive error

**Symptoms**:
- Drive not ready
- Motor not responding
- Drive status LED indicates fault

**Master Recovery**:
```
1. Read fault code (010)
2. Check drive status (if accessible)
3. Set diMotionEnable = FALSE
4. Mirror code: DI0=0, DI1=1, DI2=0
5. Pulse diFaultReset
6. If persists: may require power cycle
```

**Investigation**:
- Check drive front panel for error code
- Verify motor wiring
- Check motor temperature
- Review drive parameters

---

### FAULT_POSITION (011)

**Description**: Position exceeded software limits.

**Triggered When**:
- Actual position < `posSoftLimitMin`
- Actual position > `posSoftLimitMax`
- Position drifted outside limits during stop

**Symptoms**:
- Position near or beyond expected limits
- Motion was toward limit when stopped

**Master Recovery**:
```
1. Read fault code (011)
2. Note current position from AO0
3. Set diMotionEnable = FALSE
4. Mirror code: DI0=1, DI1=1, DI2=0
5. Pulse diFaultReset
6. After clear: command position away from limit
```

**Important**: After clearing, immediately command a safe position. The fault will retrigger if position remains out of limits.

---

### FAULT_HOMING_REQ (100)

**Description**: Operational mode attempted without homing complete.

**Triggered When**:
- Mode 001-100 commanded when `flagAbsHomeRequired = TRUE`
- Mode 001-100 commanded when `flagEOTHomeRequired = TRUE`
- Both flags are enforced for safety

**Symptoms**:
- Cannot enter operational modes
- `doHomingComplete` is LOW

**Master Recovery**:
```
1. Read fault code (100)
2. Set diMotionEnable = FALSE
3. Mirror code: DI0=0, DI1=0, DI2=1
4. Pulse diFaultReset
5. After clear: command homing modes

   If flagAbsHomeRequired (encoder invalid):
   - Command Mode 110 (Home to Limit)

   If flagEOTHomeRequired (every power-up):
   - Command Mode 111 (Home to EOT)

   Or command Mode 101 (Go Home) which auto-redirects
```

**Note**: This fault will recur if operational mode commanded again without completing homing.

---

### FAULT_PISTON_EXIT (101)

**Description**: Piston exit prevention guard triggered.

**Triggered When**:
- Position near `posSoftLimitMin` (exit boundary)
- Torque mode anomaly: commanding torque one direction, moving opposite (indicates external pressure overcoming motor)
- Guard detected dangerous approach to exit boundary

**Symptoms**:
- Motion halted near minimum position
- In torque mode: position drifting toward exit despite command

**Master Recovery**:
```
1. Read fault code (101)
2. CRITICAL: Check if cylinder is pressurized
3. Set diMotionEnable = FALSE
4. Mirror code: DI0=1, DI1=0, DI2=1
5. Pulse diFaultReset
6. After clear: command position away from exit
```

**Investigation**:
- Check cylinder pressure
- Verify piston seal integrity
- Reduce commanded torque/force
- Check for mechanical binding

**Safety Note**: This fault indicates a potentially dangerous condition. The piston was approaching or being pushed toward the cylinder exit. Investigate before resuming operation.

---

### FAULT_LIMIT_SWITCH (110)

**Description**: Unexpected limit switch activation during motion.

**Triggered When**:
- Limit switch triggered during operational mode (not homing)
- Could indicate position error, mechanical issue, or runaway

**Symptoms**:
- One of the limit switches (DI4 or DI5) went LOW unexpectedly
- Position may not match expected location

**Master Recovery**:
```
1. Read fault code (110)
2. Stop all operations
3. Set diMotionEnable = FALSE
4. Mirror code: DI0=0, DI1=1, DI2=1
5. DO NOT automatically retry
6. Requires investigation
```

**Investigation**:
- Which limit switch triggered?
- Does position make sense for that switch?
- Check for mechanical obstruction
- Verify limit switch wiring
- Check for position drift or encoder error

**Manual Recovery**:
1. Clear fault with mirrored code
2. Carefully command mode to move away from limit
3. Consider re-homing if position uncertain

---

### FAULT_ENCODER (111)

**Description**: Absolute encoder position data invalid.

**Triggered When**:
- Encoder battery failure
- Multi-turn data corruption
- Encoder communication lost (after initialization)
- AbsolutePositionManager reports invalid

**Symptoms**:
- Position feedback may be incorrect
- `flagAbsHomeRequired` was set TRUE
- Cannot trust absolute position

**Master Recovery**:
```
1. Read fault code (111)
2. Set diMotionEnable = FALSE
3. Mirror code: DI0=1, DI1=1, DI2=1
4. Pulse diFaultReset
5. After clear: MUST perform homing
   - Command Mode 110 (Home to Limit)
```

**Investigation**:
- Check encoder battery status (drive parameter)
- Verify encoder cable connection
- Check for electrical noise interference

**Important**: Never operate in position-dependent modes without valid homing after this fault.

---

## 4. Fault Reset Procedure

### Complete Reset Sequence

```
DETECT_FAULT:
    // doFaultActive = HIGH
    fault_code = (DO2 << 2) | (DO1 << 1) | DO0
    log_fault(fault_code)

PREPARE_RESET:
    // Ensure motion disabled
    SET diMotionEnable = FALSE
    WAIT 20ms  // Ensure stable

    // Mirror fault code to mode bits
    DI0 = fault_code & 0x01
    DI1 = (fault_code >> 1) & 0x01
    DI2 = (fault_code >> 2) & 0x01
    WAIT 20ms  // Ensure stable

EXECUTE_RESET:
    // Assert reset (rising edge)
    SET diFaultReset = FALSE  // Ensure low first
    WAIT 10ms
    SET diFaultReset = TRUE   // Rising edge

    // Wait for acknowledgment
    START timeout_timer (1000ms)

WAIT_CLEAR:
    IF doFaultActive = FALSE THEN
        // Fault cleared successfully
        SET diFaultReset = FALSE
        GOTO SUCCESS
    ELSIF timeout_timer expired THEN
        SET diFaultReset = FALSE
        GOTO RESET_FAILED

SUCCESS:
    // Return to idle state
    SET mode_bits = 000
    // Determine next action based on fault type

RESET_FAILED:
    // Fault persists - requires investigation
    // May need power cycle or mechanical fix
```

### Reset Timing Diagram

```
doFaultActive  ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                         ^
                   |                         |
                   Fault                     Cleared
                   detected

diMotionEnable ‾‾‾‾\________________________/‾‾‾‾
                    ^                       ^
                    |                       |
                    Disable                 Re-enable
                    for reset               after clear

mode_bits      XXXX|--- fault_code ---------|0000
                   ^                        ^
                   |                        |
                   Mirror                   Return
                   code                     to idle

diFaultReset   ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                   ^
                   |                   |
                   Rising              Release
                   edge                after clear
```

---

## 5. Recovery Decision Tree

```
FAULT DETECTED (doFaultActive = HIGH)
           |
           v
    Read fault code
           |
    +------+------+------+------+------+------+------+
    |      |      |      |      |      |      |      |
   001    010    011    100    101    110    111
    |      |      |      |      |      |      |
    v      v      v      v      v      v      v
 Retry   Check  Move   Home   Check  STOP   Home
 mode    drive  from   first  press  Invest required
         status limit         ure    igate

After reset, determine appropriate next action:

001 (Handshake) -> Retry previous mode command
010 (Drive)     -> May need power cycle, then re-home
011 (Position)  -> Command safe position immediately
100 (Homing)    -> Command Mode 110 and/or 111
101 (Piston)    -> Check pressure, move away from exit
110 (Limit)     -> Manual investigation required
111 (Encoder)   -> Must complete Mode 110 homing
```

---

## 6. Fault Prevention Guidelines

### Handshake Faults
- Ensure stable mode bits before raising motion enable
- Don't modify mode during handshake window
- Use adequate debounce on digital outputs

### Drive Faults
- Stay within motor thermal limits
- Ensure adequate power supply capacity
- Regular drive maintenance

### Position Faults
- Command positions well within limits
- Use appropriate acceleration/deceleration
- Monitor position feedback during motion

### Homing Requirement Faults
- Always complete homing after power cycle
- Check `doHomingComplete` before operational modes
- Use Mode 101 (Go Home) for automatic handling

### Piston Exit Faults
- Avoid commanding toward exit boundary under load
- Monitor cylinder pressure
- Use appropriate force limits in torque mode

### Limit Switch Faults
- Verify position tracking after homing
- Check for mechanical obstruction
- Regular limit switch testing

### Encoder Faults
- Monitor encoder battery status
- Replace battery proactively
- Ensure proper cable routing (no noise sources)

---

## 7. Quick Reference

### Fault Code Table

| Code | Bits | Name | Quick Fix |
|------|------|------|-----------|
| 0 | 000 | None | - |
| 1 | 001 | Handshake | Retry mode |
| 2 | 010 | Drive | Check drive |
| 3 | 011 | Position | Move to safe |
| 4 | 100 | Homing Req | Home axis |
| 5 | 101 | Piston Exit | Check pressure |
| 6 | 110 | Limit Switch | Investigate |
| 7 | 111 | Encoder | Home axis |

### Reset Checklist

1. [ ] diMotionEnable = FALSE
2. [ ] Mode bits = fault code (mirror)
3. [ ] Wait 20ms for stable
4. [ ] Rising edge on diFaultReset
5. [ ] Wait for doFaultActive = FALSE
6. [ ] Release diFaultReset
7. [ ] Set mode bits = 000
8. [ ] Take corrective action per fault type
