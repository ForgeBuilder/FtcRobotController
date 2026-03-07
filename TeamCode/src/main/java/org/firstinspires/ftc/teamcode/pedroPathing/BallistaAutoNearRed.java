package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BallistaAutoNearRed",preselectTeleOp = "CrossbowTeleopRed")
public class BallistaAutoNearRed extends BallistaAutoNear {
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}