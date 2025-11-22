package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CrossbowAutoBlue",preselectTeleOp = "CrossbowTeleopBlue")
public class CrossbowAutoBlue extends CrossbowAuto{
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}
