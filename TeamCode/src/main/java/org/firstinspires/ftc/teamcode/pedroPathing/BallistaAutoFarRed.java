package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BallistaAutoFarRed",preselectTeleOp = "CrossbowTeleopRed")
public class BallistaAutoFarRed extends BallistaAutoFar {
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}