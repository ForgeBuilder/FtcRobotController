package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;


//This is the auto program. a few values will be able to be change to make it work for red/blue
//or near/far. near/far might be completley different programs based on how much crossbowMain allready abstracts
//but I allways could inherit the class bwahaha
public class CrossbowAuto extends CrossbowMain{
    private ElapsedTime runtime = new ElapsedTime();

    //auto position multiplier. flips the auto for red and blue. //Should only be 1 or -1
    private double apm;
    //It's a double so the rotation plays nice.
    @Override
    public void set_team(String team) {
        super.set_team(team);
        if (team == "red"){
            apm = -1.0;
        } else if (team == "blue"){
            apm = 1.0;
        }
    }

    @Override public void init(){
        super.init();
        set_launcher_speed(620);
    }

    @Override public void start(){
        super.start();
        follower.setPose(new Pose(101,-7.5*apm, apm*(Math.PI/2)));
        runtime.reset();
    }

    private int fired_artifacts = 0;
    private boolean fired_an_artifact = false;
    private boolean fire_artifact = false;
    

    private int step = 0;

    private int intake_round = 0;

    double drivetrain_pickup_speed = 0.4;

    double resume_time = 200;

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
            Pose launch_pose;
            if (intake_round == 0){
                launch_pose = new Pose(100,-40*apm,1.2*apm);
            } else {
                //barely touch the line
                launch_pose = new Pose(72,-40*apm,0.65*apm);
            }

            PathChain firstpath = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, launch_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), launch_pose.getHeading(),0.5)
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
            Pose next_pose = new Pose(74,-40*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
//                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 4;
        } else if (step == 4 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(74,-13*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 5;
        } else if ((step == 5) && !follower.isBusy()){
//            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 1;
            step = 0;
        } else if (step == 6 && steptimer.seconds() > 0.5){
            //go to intake bar 2
            fire_artifact = false;
            Pose next_pose = new Pose(51.5,-40*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
//                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 7;
        } else if (step == 7 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(51.5,-13*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            Pose avoid_gate_pose = new Pose(51.5,-30*apm,apm*(Math.PI/-2.0));
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .addPath(new BezierLine(next_pose, avoid_gate_pose))
                    .addParametricCallback(1, () -> { // Pause 80% through the *previous* path
                        follower.pausePathFollowing(); // Stop robot movement
                        resume_time = runtime.seconds()+(double) 0.3;
                        //0.3 is how long the robot will wait to ensure it intakes all artifacts. 0.3 is 1/100 of the time aloted for auto.
                    })
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 8;
        } else if (step == 8 && !follower.isBusy()){
//            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 2;
            step = 0;
        } else if (step == 9 && steptimer.seconds() > 0.5){
            //go to intake bar 3
            fire_artifact = false;
            Pose next_pose = new Pose(28,-40*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
//                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
            step = 10;
        } else if (step == 10 && !follower.isBusy()){
            //slowly roll over to pickup balls
            spin_intake = true;
            Pose next_pose = new Pose(28,-13*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .addPath(new BezierLine(next_pose, current_pose))
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 11;
            //THE END sort of
        } else if (step == 11 && !follower.isBusy()){
//            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round = 2;
            //don't take the last shot we'll be on the bar.
//            step = 0;
        }


        //if the match is about to end, get off the launch line!!

        //check where we are and depending on which side of the launch line we are on, go to a different spot.

       //I HAVE NOT IMPLEMENTED THE ABOVE, DO IT WHEN YOU COME BACK PLEASE!

        if (runtime.seconds() > resume_time){
            resume_time = 200;
            follower.resumePathFollowing();
        }

        if (runtime.seconds() > 28.0){
            //go to intake bar 1
            step = 100;
            fire_artifact = false;
            Pose next_pose = new Pose(100,-40*apm,apm*1.2);
            Pose current_pose = follower.getPose();
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .setHeadingConstraint(0)
                    .build();
            follower.followPath(center_path);
        }

        telemetry.addData("fired artifacts: ",fired_artifacts);
        telemetry.addData("step",step);
        telemetry.update();
        panelsTelemetry.update(telemetry);
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
