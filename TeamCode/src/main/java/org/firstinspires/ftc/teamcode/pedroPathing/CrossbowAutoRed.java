package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="tRed",preselectTeleOp = "CrossbowTeleopRed")
public class CrossbowAutoRed extends CrossbowAuto{
    @Override
    public void init(){
        set_team("red");
        super.init();
    }
}
