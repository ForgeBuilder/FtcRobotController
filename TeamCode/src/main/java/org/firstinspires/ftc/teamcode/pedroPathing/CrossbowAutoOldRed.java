package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoRed",preselectTeleOp = "CrossbowTeleopRed")
public class CrossbowAutoOldRed extends CrossbowAutoOld {
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}
