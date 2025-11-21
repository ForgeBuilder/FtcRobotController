package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="CrossbowAutoRed",preselectTeleOp = "CrossbowTeleopRed")
public class CrossbowAutoRed extends CrossbowAuto{
    @Override
    public void init(){
        super.init();
        set_team("red");
    }
}
