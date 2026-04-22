G_CmdPower.Enable:=G_diFaultReset;
G_CmdPower.EnablePositive:=TRUE;
G_CmdPower.EnableNegative:=TRUE;

   
G_doBrakeDisengage:=G_StaPower.Status;
Error:=G_StaPower.Error;