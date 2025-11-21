package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CrossbowTeleopRed")
public class CrossbowTeleopRed extends CrossbowTeleop{
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}
