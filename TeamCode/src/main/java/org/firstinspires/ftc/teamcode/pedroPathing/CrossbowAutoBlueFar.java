package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoBlueFar",preselectTeleOp = "CrossbowTeleopBlue")
public class CrossbowAutoBlueFar extends CrossbowAutoFar{
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}
