# MC_SetPosition Function Block Verification

## Purpose
This document verifies the MC_SetPosition function block usage in PRG_Main.st against Yaskawa MotionWorksIEC and PLCopen specifications.

## Verification Date
2025-12-29

## Sources Consulted

### Primary Sources
- [Yaskawa MotionWorks IEC User Manual - MC_SetPosition (Page 154)](https://www.manualsdir.com/manuals/802015/yaskawa-motionworks-iec.html?page=154)
- [PLCopen Plus Function Blocks for Motion Control (YEA-SIA-IEC-3)](https://www.yaskawa.com/downloads/search-index/details?showType=details&docnum=YEA-SIA-IEC-3)
- [Beckhoff MC_SetPosition Reference](https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70052491.html)

### Additional References
- [PLCopen Motion Control Function Block Reference](https://infosys.beckhoff.com/content/1033/tcplclib_tc2_mc2/70043531.html)

---

## MC_SetPosition Interface Specification

### VAR_IN_OUT
| Parameter | Type | Description |
|-----------|------|-------------|
| Axis | AXIS_REF | Logical axis reference from Hardware Configuration |

### VAR_INPUT
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| Execute | BOOL | Rising edge triggers the function | FALSE |
| Position | LREAL | Target position value in user units | 0.0 |
| Mode | BOOL | FALSE = ABSOLUTE (default), TRUE = RELATIVE | FALSE |

### VAR_OUTPUT
| Parameter | Type | Description |
|-----------|------|-------------|
| Done | BOOL | TRUE when position change completed successfully |
| Busy | BOOL | TRUE while function block is executing |
| Error | BOOL | TRUE if error occurred |
| ErrorID | UDINT/DWORD | Error identification number |

---

## Current Implementation Analysis

### Code Locations
- **ST_HOME_LIM_SETREF** (PRG_Main.st lines 764-768):
```st
fbSetPosition(
    Axis := sysAxis,
    Execute := TRUE,
    Position := cfgHomeLimSetPosition
);
```

- **ST_HOME_EOT_SETREF** (PRG_Main.st lines 954-958):
```st
fbSetPosition(
    Axis := sysAxis,
    Execute := TRUE,
    Position := cfgHomeEOTSetPosition
);
```

### Assessment

**Mode Parameter Omission**: The implementation does not explicitly specify the `Mode` parameter.

**Impact**: **NONE - Implementation is CORRECT**

The Mode parameter defaults to FALSE (ABSOLUTE mode), which is the correct behavior for homing operations:
- After homing to limit switch (Mode 110), we set absolute position to 0.0mm
- After homing to EOT (Mode 111), we set absolute position to 300.0mm
- Both operations require ABSOLUTE mode to establish the position reference

### Conclusion
The current implementation is **functionally correct**. The omission of the Mode parameter is acceptable because:
1. The default value (FALSE = ABSOLUTE) is the intended behavior
2. PLCopen function blocks are designed with sensible defaults
3. The implementation follows standard PLCopen patterns

---

## Recommendation

For improved code clarity and maintainability, consider explicitly specifying the Mode parameter:

```st
fbSetPosition(
    Axis := sysAxis,
    Execute := TRUE,
    Position := cfgHomeLimSetPosition,
    Mode := FALSE  (* ABSOLUTE - set position reference *)
);
```

This is an optional enhancement for documentation purposes, not a required fix.

---

## PLCopen Function Block Behavior Notes

### Output State Rules
- Done, Busy, Error, and CommandAborted are mutually exclusive
- Only one of these outputs can be TRUE at a time
- Busy is SET on rising edge of Execute
- Busy is RESET when Done, Error, or CommandAborted becomes TRUE

### Error Handling
- Error and ErrorID are reset with falling edge of Execute
- Function block errors do not require explicit reset
- The falling edge of Execute does not stop execution

---

## Document History
| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-29 | Claude Code | Initial verification |
