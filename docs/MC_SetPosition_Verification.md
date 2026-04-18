# Position Reference Setting — Current Implementation

## Purpose
This document describes how the absolute position reference is established during homing. It supersedes an earlier verification document that reviewed direct use of PLCopen `MC_SetPosition` from `PRG_Main`.

## Summary of the Change

`MC_SetPosition` is **no longer called from `PRG_Main`**. Position reference setting now flows through `AbsolutePositionManager`, which is instantiated in the Ladder Diagram POU and commanded through the `G_cmdEncMngr` / `G_staEncMngr` globals.

The two homing modes set position differently:

- **Mode 110 (Home to Limit Switch)** — `FB_HomeLimit` drives `CmdEncMngr.SetPosition := TRUE` with the target absolute position (`G_cfgHomeLimSetPosition`, default `0.0 mm`). `PRG_Main` copies this command output into `G_cmdEncMngr`; the LD POU's `AbsolutePositionManager` performs the actual position-reference write and reports back via `G_staEncMngr.SetPositionDone`.
- **Mode 111 (Home to End-of-Travel)** — `FB_HomeEOT` **does not set position**. Instead, once stall is confirmed it records the encoder position (`EOTPosition := ActualPosition`) and computes a master/slave coordinate offset (`EOTOffset := ExpectedEOTPosition - EOTPosition`). This preserves the encoder zero reference established by Mode 110.

## Data Flow

```
FB_HomeLimit (ST, HL_SETREF state)
     │
     │  CmdEncMngr.Enable := TRUE
     │  CmdEncMngr.SetPosition := TRUE
     │  CmdEncMngr.Position := SetPosition   (= G_cfgHomeLimSetPosition)
     ▼
PRG_Main routes fbHomeLimit.CmdEncMngr -> G_cmdEncMngr
     ▼
Ladder Diagram POU
     AbsolutePositionManager instance
     (reads G_cmdEncMngr, writes G_staEncMngr)
     ▼
G_staEncMngr.SetPositionDone  -> back into FB_HomeLimit.StaEncMngr
     │
     └─► FB_HomeLimit transitions HL_SETREF -> HL_DONE
```

## Why AbsolutePositionManager instead of `MC_SetPosition`

`AbsolutePositionManager` is the Yaskawa-provided block for the Sigma-7 absolute encoder. It:

1. Validates multi-turn / battery-backed encoder state (`Valid`, `PositionValid` outputs).
2. Writes the position reference at the home location.
3. Reports operation status through `SetPositionDone` and `Error`.

Using it consolidates "encoder validity check" and "set position reference" behind a single block, which is a better fit for the Sigma-7 absolute encoder workflow than PLCopen `MC_SetPosition`.

## Coordinate System After Homing

| Situation                        | Encoder zero   | Master sees                             |
|----------------------------------|----------------|------------------------------------------|
| After Mode 110 only              | At home switch | Master position = encoder position       |
| After Mode 110 then Mode 111     | At home switch | Master position = encoder position + `G_posEOTOffset` |

`G_posEOTOffset` is applied in `FB_PositionOutput` when generating the analog feedback (AO0) and when translating master position commands in position-control mode.

## Where the Logic Lives

| Concern                         | Location (current)                         |
|---------------------------------|--------------------------------------------|
| Position reference write        | `AbsolutePositionManager` in the LD POU    |
| Command emission (Mode 110)     | `FB_HomeLimit.st`, state `HL_SETREF`       |
| Status receipt                  | `FB_HomeLimit` input `StaEncMngr`          |
| ST ↔ LD bridge                  | `G_cmdEncMngr`, `G_staEncMngr`             |
| EOT offset calculation          | `FB_HomeEOT.st`, state `HE_SETREF`         |

## Document History
| Version | Date       | Changes |
|---------|------------|---------|
| 1.0     | 2025-12-29 | Initial verification (MC_SetPosition direct call) |
| 2.0     | 2026-04-17 | Rewritten — position reference now set via `AbsolutePositionManager` / `G_cmdEncMngr`; `MC_SetPosition` no longer used from `PRG_Main` |
