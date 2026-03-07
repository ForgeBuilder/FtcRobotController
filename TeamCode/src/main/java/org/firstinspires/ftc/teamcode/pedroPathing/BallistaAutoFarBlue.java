package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BallistaAutoFarBlue",preselectTeleOp = "CrossbowTeleopBlue")
public class BallistaAutoFarBlue extends BallistaAutoFar {
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}