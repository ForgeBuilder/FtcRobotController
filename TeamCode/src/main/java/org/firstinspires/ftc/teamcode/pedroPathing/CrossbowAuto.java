package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    private boolean fire_artifact = false;

    private int step = 0;

    @Override public void loop(){
        intake_code();
        fired_an_artifact = launcher_code(fire_artifact,false);


        if (step == 0){
            Pose startpose = new Pose(0,0,0);
            Pose endpose = new Pose(20,0,0);
            PathChain firstpath = follower.pathBuilder()
                    .addPath(new BezierLine(startpose, endpose))
                    .setLinearHeadingInterpolation(startpose.getHeading(), endpose.getHeading())
                    .build();
            step = 1;
        } else if (!follower.isBusy() && (step ==1)){
            //get the limelight to set the pose
//            follower.setPose()
            step = 2;
        } else if (step == 2) {
            if (fired_artifacts <= 3){
                if (fired_an_artifact){
                    fired_artifacts += 1;
                    if (fired_artifacts >= 3){
                        spin_intake = false;
                        fire_artifact = false;
                    }
                }
            }
        }

        if (follower.isBusy()) {
            follower_code(false);
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
