## Notes on GVL_Config.st
On position mapping, there are 1) redundant variables (cfgFeedbackStage1VoltMax and  cfgFeedbackStage2VoltMin, etc.) and 2) "Position reference scaling" and "Position feedback - Two-stage scaling for improved resolution". For 1, avoid redundancy. set the transition between stages and 2, get rid of none 2-stage position mapping.
## Notes on PRG_Main.st
the handshake in ST_IDLE does not look to be implemented properly. It seems there is other code handling the handshake. Or at least, the results of fbHandshake arn't being used.
we need homing-to-limit-switch to be required before enabling EOT. this is counter to logic and comment in lines 274-277 in PRG_Main.st
Redundant lines 293 and 298
Line 339 - default state should be ST_HOLD_POSITION rather than ST_IDLE if the brake is being released
line 358 - ST_BRAKE_HOLD is a special case where we stay in that state if motion enable drop (it is safer and more efficient to just eave the brake engaged)
how is sysConfirmMode used? (ex. line 364 and others) Is it intended for functionality to be implemented? If so, we need temp comments. 
line 390, 422, 453 (and others?) - make threshold a cfg variable
Lines 416 - double check if Y_DirectControl include limit position, especially in Velocity control mode
Should Line 478 move into IF-ELSE pattern of lines 482-486? Same with fbHalt of ST_HOLD_POSITION?
what is flagHomingInProgress used for? is it relevant when fTrigMotionEnable.Q = TRUE and the state changes without changing the flag?
Any issue with fbHalt(Execute:=TRUE) before fbDirectControl(Execute:FALSE?)
in ST_HOLD_POSITION, it looks like the state could change before a halt is complete. That's fine. I just want to verify that I understand what is happening.
On ST_FAULT (line 651): I think fault states should be less dramatic. We don't need to do a E-Stop. We just need to notify and wait patiently. This should be like ST_BRAKE_HOLD or ST_IDLE (I would prefer ST_IDLE but maybe we implement a timeout to go from ST_BRAKE_HOLD to ST_IDLE to give us the opportunity for faster resets without just leaving the drive on perpetually). We should use MC_Halt instead of MC_Stop. Then engage the brake. Then shut down the drive (maybe after a timeout).
For the fault reset handshake, I want the master to mirror th fault code before the slave will acknowledge a fault reset command. (maybe that is what is implemented)
Line 719 - fbFaultOutput (and FB_FaultCodeOutput) needs to be renamed. It serves both mode confirm and fault code output. It shouldn't only be names after one or the other.