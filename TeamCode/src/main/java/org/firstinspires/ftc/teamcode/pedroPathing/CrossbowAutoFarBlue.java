package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoFarBlue",preselectTeleOp = "CrossbowTeleopBlue")
public class CrossbowAutoFarBlue extends CrossbowAutoFar {
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}
