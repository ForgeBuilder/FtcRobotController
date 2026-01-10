package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoRedFar",preselectTeleOp = "CrossbowTeleopRed")
public class CrossbowAutoRedFar extends CrossbowAutoFar{
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}
