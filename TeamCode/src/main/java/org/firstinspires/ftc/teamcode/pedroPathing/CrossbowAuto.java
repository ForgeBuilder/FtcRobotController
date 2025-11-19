package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


//This is the auto program. a few values will be able to be change to make it work for red/blue
//or near/far. near/far might be completley different programs based on how much crossbowMain allready abstracts
//but I allways could inherit the class bwahaha

@Autonomous(name="CrossbowAuto")

public class CrossbowAuto extends CrossbowMain{
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void init(){
        super.init();
    }

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
            Pose endpose = new Pose(40,0,0);
            PathChain firstpath = follower.pathBuilder()
                    .addPath(new BezierLine(startpose, endpose))
                    .setLinearHeadingInterpolation(startpose.getHeading(), endpose.getHeading())
                    .build();
            follower.followPath(firstpath);
            step = 1;
        } else if (!follower.isBusy() && (step ==1)){
            //get the limelight to set the pose
//            follower.setPose()
            step = 2;
            fire_artifact = true;
            set_limelight_enabled(true);
        } else if (step == 2) {

            limelight_set_botpose();

            if (fired_artifacts <= 3){
                if (fired_an_artifact){
                    fired_artifacts += 1;
                    if (fired_artifacts >= 3){
                        spin_intake = false;
                        fire_artifact = false;
                        step = 3;
                        Pose center_pose = new Pose(0,0,0);
                        Pose current_pose = follower.getPose();
                        PathChain center_path = follower.pathBuilder()
                                .addPath(new BezierLine(current_pose, center_pose))
                                .setLinearHeadingInterpolation(current_pose.getHeading(), center_pose.getHeading())
                                .build();
                        follower.followPath(center_path);
                    }
                }
            }
        } else if (step == 3 ) {

        }

        if (true) { //follower.isBusy()
            follower_code(false);
        }

        telemetry.addData("fired artifacts: ",fired_artifacts);
        telemetry.addData("step",step);
        telemetry.update();
    }
    private boolean spin_intake = true;

    public void set_limelight_enabled (boolean enabled){
        if (enabled){
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
            limelight.pipelineSwitch(9); // Switch to pipeline number 0
        } else {
            limelight.stop(); // This tells Limelight to stop looking.
        }
    }

    public void limelight_set_botpose(){
        LLresult = limelight.getLatestResult();
        if (LLresult != null && LLresult.isValid()) {
            Pose3D limelight_botpose = LLresult.getBotpose();
            if (limelight_botpose != null) {
                double x = limelight_botpose.getPosition().x;
                double y = limelight_botpose.getPosition().y;
                YawPitchRollAngles limelight_orientation = limelight_botpose.getOrientation();
                double yaw = limelight_orientation.getYaw(AngleUnit.RADIANS);
                double meters_to_inches = 39.3701;
                telemetry.addData("MT1 Location", "(" + x*meters_to_inches + ", " + y*meters_to_inches + ")");
                telemetry.addData("MT1 Yaw", yaw);

                //this will likley be very off becasue the limelight is backwards..
                //the negitives and math.pi are to reverse the pose
                Pose limelight_pose = new Pose(-x*meters_to_inches,y*meters_to_inches,yaw-Math.PI);
                follower.setPose(limelight_pose);
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    //doesnt return anything yet, not used yet.
    public void backboard_scan_simple(){
        LLresult = limelight.getLatestResult();
        if (LLresult != null && LLresult.isValid()) {
            double tx = LLresult.getTx(); // How far left or right the target is (degrees)
            double ty = LLresult.getTy(); // How far up or down the target is (degrees)
            double ta = LLresult.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
//            telemetry.addData("Target Y", ty);
//            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
    @Override public void intake_code(){
        if (spin_intake&&(!kick)){
            set_intake_speed(2000);
        } else {
            set_intake_speed(0);
        }
    }
}
