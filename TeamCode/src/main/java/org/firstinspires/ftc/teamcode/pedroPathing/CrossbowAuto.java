package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


//This is the auto program. a few values will be able to be change to make it work for red/blue
//or near/far. near/far might be completley different programs based on how much crossbowMain allready abstracts
//but I allways could inherit the class bwahaha

@Autonomous(name="CrossbowAutoBlue",preselectTeleOp = "CrossbowTeleop")

public class CrossbowAuto extends CrossbowMain{
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void init(){
        super.init();
        follower.setPose(new Pose(101,-7.5,Math.PI/2));
        set_launcher_speed(620);
    }

    @Override public void start(){
        super.start();
        runtime.reset();
    }

    private int fired_artifacts = 0;
    private boolean fired_an_artifact = false;
    private boolean fire_artifact = false;

    private int step = 0;

    private int intake_round = 0;

    double drivetrain_pickup_speed = 0.5;

    private ElapsedTime steptimer = new ElapsedTime();

    @Override public void loop(){
        super.loop();
        intake_code();
        fired_an_artifact = launcher_code(fire_artifact,false);

        Pose current_posee = follower.getPose();
        telemetry.addData("Pedro Pose: ",current_posee.getX()+", "+current_posee.getY()+", "+current_posee.getHeading());

        if (step == 0){
            //go to the launching position
            Pose current_pose = follower.getPose();

            Pose next_pose = new Pose(90,-45,1);
            PathChain firstpath = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .build();
            follower.followPath(firstpath);
            step = 1;
        } else if (!follower.isBusy() && (step ==1)){
            //get the limelight to set the pose
//            follower.setPose()
            step = 2;
            fire_artifact = true;
            spin_intake = true;
            set_limelight_enabled(true);
        } else if (step == 2) {

//            limelight_set_pose();

            if (fired_artifacts <= 3){
                if (fired_an_artifact){
                    fired_artifacts += 1;
                    if (fired_artifacts >= 3){
                        spin_intake = false;
                        steptimer.reset();
                        if (intake_round == 0){
                            step = 3;
                        } else if (intake_round == 1){
                            step = 6;
                        } else if (intake_round == 2) {
                            step = 9;
                        }
                    }
                }
            }
        } else if (step == 3 && steptimer.seconds() > 0.5) {
            //go to intake bar 1
            fire_artifact = false;
            Pose next_pose = new Pose(73,-40,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 4;
        } else if (step == 4 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(73,-12,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 5;
        } else if ((step == 5) && !follower.isBusy()){
            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 1;
            step = 0;
        } else if (step == 6 && steptimer.seconds() > 0.5){
            //go to intake bar 2
            fire_artifact = false;
            Pose next_pose = new Pose(51,-40,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 7;
        } else if (step == 7 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(51,-12,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 8;
        } else if (step == 8 && !follower.isBusy()){
            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 2;
            step = 0;
        } else if (step == 9 && steptimer.seconds() > 0.5){
            //go to intake bar 3
            fire_artifact = false;
            Pose next_pose = new Pose(37,-40,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 10;
        } else if (step == 10 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(37,-12,Math.PI/-2.0);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 11;
            //THE END sort of
        } else if (step == 11 && !follower.isBusy()){
            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 2;
            step = 0;
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
        } else {
            limelight.stop(); // This tells Limelight to stop looking.
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
