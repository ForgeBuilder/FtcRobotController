package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BallistaAutoNearBlue",preselectTeleOp = "CrossbowTeleopBlue")
public class BallistaAutoNearBlue extends BallistaAutoNear {
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}