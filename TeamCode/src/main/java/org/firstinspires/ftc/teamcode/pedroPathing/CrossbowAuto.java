package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


//This is the auto program. a few values will be able to be change to make it work for red/blue
//or near/far. near/far might be completley different programs based on how much crossbowMain allready abstracts
//but I allways could inherit the class bwahaha

@Autonomous(name="CrossbowAuto")

public class CrossbowAuto extends CrossbowMain{
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void start(){
        super.start();
        runtime.reset();
    }

    private int fired_artifacts = 0;
    private boolean fired_an_artifact = false;

    @Override public void loop(){
        intake_code();
        if (fired_artifacts <= 3){
            fired_an_artifact = launcher_code(true,false);
            if (fired_an_artifact){
                fired_artifacts += 1;
                if (fired_artifacts >= 3){
                    spin_intake = false;
                }
            }
        }
        telemetry.addData("fired artifacts: ",fired_artifacts);
        telemetry.update();
    }
    private boolean spin_intake = true;
    @Override public void intake_code(){
        if (spin_intake&&(!kick)){
            set_intake_speed(2000);
        } else {
            set_intake_speed(0);
        }
    }
}
