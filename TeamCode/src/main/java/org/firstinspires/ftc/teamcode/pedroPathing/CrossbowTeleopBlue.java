package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CrossbowTeleopBlue")
public class CrossbowTeleopBlue extends CrossbowTeleop{
    @Override
    public void init(){
        super.init();
        set_team("blue");
    }
}
