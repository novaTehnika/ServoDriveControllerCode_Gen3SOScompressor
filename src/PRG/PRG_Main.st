(* ============================================================================
   PRG_Main - Main Program (Incremental Bring-Up)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Incremental bring-up to isolate the MotionWorks IEC "Error during generating
   native code!" failure. See /home/jeremy/.claude/plans/we-have-a-seemingly-
   breezy-cloud.md for phase plan. Each phase adds one FB/FN (or tightly
   coupled pair) and must compile before moving on.

   Current phase: Phase 1 Step 1 - add FN_IsOperationalMode + FN_IsHomingMode

   Variables (define in MWiec GUI):
     VAR:
       (* Phase 1 Step 1 - FN result latches
       bIsOperational      : BOOL
       bIsHoming           : BOOL
	   fb_IsOperationalMode : FB_IsOperationalMode
   ============================================================================ *)

(* MC_Power passthrough - baseline *)
G_CmdPower.Enable         := G_diFaultReset;
G_CmdPower.EnablePositive := TRUE;
G_CmdPower.EnableNegative := TRUE;

G_doBrakeDisengage := G_StaPower.Status;
Error              := G_StaPower.Error;

(* Phase 1 Step 1: exercise FN_IsOperationalMode *)
G_sysRequestedMode := E_OperatingMode#MODE_IDLE;

(* Approach 1: fail *)
(*bIsOperational := FN_IsOperationalMode(G_sysRequestedMode);*)

(* Approach 2: pass *)
bIsOperational := FN_AlwaysTrue_BOOL(TRUE);

(* Approach 3: fail *)
(*bIsOperational := FN_AlwaysTrue_ENUM(G_sysRequestedMode);*)

(* Approach 4: pass *)
fb_IsOperationalMode(Mode:=G_sysRequestedMode);
bIsOperational := fb_IsOperationalMode.Ans;

G_doInMotion := bIsOperational;
G_doHomingComplete    := bIsHoming;
