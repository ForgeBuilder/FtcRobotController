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
        set_launcher_speed(780);
        //run the loop once to get things ready
    }

    private void go_to_gate(){
        follower.breakFollowing();
        Pose current_pose = follower.getPose();
        fire_artifact = false;
        Pose next_pose = new Pose(60,-15*apm,(Math.PI/2)*apm);
        PathChain center_path = follower.pathBuilder()
                .addPath(new BezierLine(current_pose, next_pose))
                .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                .setHeadingConstraint(0)
                .addParametricCallback(1,() ->{
                    spin_intake = false;
                })
                .build();
        follower.followPath(center_path);
    }

    public void go_to_launch(Runnable next_path){
        Pose current_pose = follower.getPose();
        Pose launch_pose = new Pose(84,-44*apm,0.9*apm);

        PathChain to_launch_path = follower.pathBuilder()
                //go to launch
                .addPath(new BezierLine(current_pose, launch_pose))
                .setLinearHeadingInterpolation(current_pose.getHeading(), launch_pose.getHeading(),0.5)
                //fire preload artifacts
                .addParametricCallback(0.99,()-> {
                    fire_artifact = true;
                    spin_intake = true;
                    set_limelight_enabled(true);
                    follower.pausePathFollowing();
                })
                .addCallback(()->follower.)
                .build();
        follower.followPath(to_launch_path);
    }

    public void go_to_bar_1(){
        Pose current_pose = follower.getPose();
        Pose intake_1_start_pose = new Pose(74,-40*apm,apm*(Math.PI/-2.0));
        Pose intake_1_end_pose = new Pose(74,-13*apm,apm*(Math.PI/-2.0));

        PathChain to_bar_1_path = follower.pathBuilder()
        //go to intake bar 1
                .addPath(new BezierLine(current_pose, intake_1_start_pose))
                .setLinearHeadingInterpolation(current_pose.getHeading(),intake_1_start_pose.getHeading(),0.5)
                //rollover bar 1
                .addPath(new BezierLine(intake_1_start_pose,intake_1_end_pose))
                .setLinearHeadingInterpolation(intake_1_start_pose.getHeading(), intake_1_end_pose.getHeading(),0.1)
                .addParametricCallback(0,()->{
                    spin_intake = true;
                    follower.setMaxPower(0.5);
                })
                .addParametricCallback(0.99,()->{
                    follower.setMaxPower(1);
                })
                .build();
        follower.followPath(to_bar_1_path);
    }

    @Override public void start(){
        super.start();
        follower.setPose(new Pose(101,-7.5*apm, apm*(Math.PI/2)));
        runtime.reset();
        loop();



        Pose intake_2_start_pose = new Pose(50.5,-40*apm,apm*(Math.PI/-2.0));
        Pose intake_2_end_pose = new Pose(50.5,-13*apm,apm*(Math.PI/-2.0));
        Pose intake_2_avoid_gate_pose = new Pose(51.5,-20*apm,apm*(Math.PI/-2.0));

        Pose intake_3_start_pose = new Pose(28,-40*apm,apm*(Math.PI/-2.0));
        Pose intake_3_end_pose = new Pose(28,-13*apm,apm*(Math.PI/-2.0));

        PathChain firstpath = follower.pathBuilder()

                //go to launch
                .addPath(new BezierLine(intake_1_end_pose,launch_pose))
                .setLinearHeadingInterpolation(intake_1_end_pose.getHeading(), launch_pose.getHeading(),0.5)
                //value of 1 does it at the end, 0 at the start, others inbetween.
                .addParametricCallback(0.99,()-> {
                    fire_artifact = true;
                    spin_intake = true;
                    set_limelight_enabled(true);
                    follower.pausePathFollowing();
                })
                //go to bar 2
                .addPath(new BezierLine(launch_pose,intake_2_start_pose))
                .setLinearHeadingInterpolation(launch_pose.getHeading(),intake_2_start_pose.getHeading(),0.5)
                //intake bar 2
                .addPath(new BezierLine(intake_2_start_pose,intake_2_end_pose))
                .setLinearHeadingInterpolation(intake_2_start_pose.getHeading(),intake_2_end_pose.getHeading(),0.1)
                .addParametricCallback(0,()->{
                    spin_intake = true;
                    follower.setMaxPower(0.5);
                })
                .addParametricCallback(0.99,()->{
                    follower.setMaxPower(1);
                })
                .addPath(new BezierLine(intake_2_end_pose,intake_2_avoid_gate_pose))
                //I should use a bezier curve here to avoid the gate
                .setLinearHeadingInterpolation(intake_2_end_pose.getHeading(),intake_2_avoid_gate_pose.getHeading(),1)
                //go to launch
                .addPath(new BezierLine(intake_2_avoid_gate_pose, launch_pose))
                .setLinearHeadingInterpolation(intake_2_avoid_gate_pose.getHeading(), launch_pose.getHeading(),0.5)
                //fire preload artifacts
                .addParametricCallback(0.99,()-> {
                    fire_artifact = true;
                    spin_intake = true;
                    set_limelight_enabled(true);
                    follower.pausePathFollowing();
                })
                //go to bar 3
                .addPath(new BezierLine(launch_pose,intake_3_start_pose))
                .setLinearHeadingInterpolation(launch_pose.getHeading(),intake_3_start_pose.getHeading(),0.5)
                //intake bar 3
                .addPath(new BezierLine(intake_3_start_pose,intake_3_end_pose))
                .setLinearHeadingInterpolation(intake_3_start_pose.getHeading(),intake_3_end_pose.getHeading(),0.1)
                .addParametricCallback(0,()->{
                    spin_intake = true;
                    follower.setMaxPower(0.5);
                })
                .addParametricCallback(0.99,()->{
                    follower.setMaxPower(1);
                })
                //go to launch
                .addPath(new BezierLine(intake_3_end_pose, launch_pose))
                .setLinearHeadingInterpolation(intake_3_end_pose.getHeading(), launch_pose.getHeading(),0.5)
                //fire preload artifacts
                .addParametricCallback(0.99,()-> {
                    fire_artifact = true;
                    spin_intake = true;
                    set_limelight_enabled(true);
                    follower.pausePathFollowing();
                })
                //literly just a piggyback for the parametric callback
                .addPath(new BezierLine(follower.getPose(),follower.getPose()))
                .addParametricCallback(0.99,()->{
                    go_to_gate();
                })
                .build();
        follower.followPath(firstpath);
    }

    private int fired_artifacts = 0;
    private boolean fired_an_artifact = false;
    private boolean fire_artifact = false;

    private int intake_round = 0;

    double drivetrain_pickup_speed = 0.4;

    double resume_time = 200;

    //how long until the system goes insane and takes the shot even if it's not ready
    //the amount of time the robot can take to make a shot until it is force to take the shot reguarless of readiness
    private double insanity_time = 4.0;

    private boolean launch_override = false;

    @Override public void loop(){
        super.loop();
        limelight_code();
        intake_code();

        //launcher stuff start

        //this if must go before launcher_code()
        if (fire_artifact) {set_motor_power_zero();}
        //this runs launcher code. only tried to launch if fire_artifact is true, but handels other launcher behavior if false.
        launch_override = (timeSinceShot.seconds() > insanity_time);
        fired_an_artifact = launcher_code(fire_artifact,launch_override);

        if (fired_an_artifact) {
            fired_artifacts += 1;
        }
        if (fire_artifact){
            if ((fired_artifacts >= 3)&&timeSinceShot.seconds() > 1.5) {
                spin_intake = false;
                fire_artifact = false;
                fired_artifacts = 0;
                follower.resumePathFollowing();
            }
        }

        //launcher stuff end
        //if the match is about to end, go to the gate to get ready and get off the launch line
        if (runtime.seconds() > 28.0){
            //this will cancel whatever part of the auto is currently happening
            go_to_gate();
        }

        Pose current_pose_telemetry = follower.getPose();
        telemetry.addData("Pedro Pose: ",current_pose_telemetry.getX()+", "+current_pose_telemetry.getY()+", "+current_pose_telemetry.getHeading());

        telemetry.addData("fired artifacts: ",fired_artifacts);
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
