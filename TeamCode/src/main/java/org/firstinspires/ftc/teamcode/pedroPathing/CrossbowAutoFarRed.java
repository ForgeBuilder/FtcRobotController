package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoFarRed",preselectTeleOp = "CrossbowTeleopRed")
public class CrossbowAutoFarRed extends CrossbowAutoFar {
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}
