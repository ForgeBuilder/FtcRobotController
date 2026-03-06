package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoBlue",preselectTeleOp = "CrossbowTeleopBlue")
public class CrossbowAutoOldBlue extends CrossbowAutoOld {
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}
