package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;


//This is the auto program. a few values will be able to be change to make it work for red/blue
//or near/far. near/far might be completley different programs based on how much crossbowMain already abstracts
//but I always could inherit the class bwahaha
public class CrossbowAutoFar extends CrossbowAuto{
//    @Override public void init(){
//        super.init();
//        set_launcher_speed(near_shot_speed);
//    }

    @Override public void start(){
        super.start();
        follower.setPose(new Pose(0,-52*apm, Math.toRadians(0)));
        runtime.reset();
    }
    @Override public void custom_auto_loop(){
        if (runtime.seconds() > 28.0){
            Pose current_pose = follower.getPose();
            if ((step == 0)||(step == 1)||(step == 2)){
                step = 100;
                fire_artifact = false;
                Pose next_pose = new Pose(60,-15*apm,(Math.PI/2)*apm);
                PathChain center_path = follower.pathBuilder()
                        .addPath(new BezierLine(current_pose, next_pose))
                        .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                        .setHeadingConstraint(0)
                        .build();
                follower.followPath(center_path);
            }
        }

        if (step == 0){
            //go to the launching position
            Pose current_pose = follower.getPose();
            Pose launch_pose;
            launch_pose = new Pose(84,-44*apm,1*apm);

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
            intake_round += 1;
            step = 0;
        } else if (step == 6 && steptimer.seconds() > 0.5){
            //go to intake bar 2
            fire_artifact = false;
            Pose next_pose = new Pose(50.5,-40*apm,apm*(Math.PI/-2.0));
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
            Pose next_pose = new Pose(49.5,-6*apm,apm*(Math.PI/-2.0));
            Pose current_pose = follower.getPose();
            Pose avoid_gate_pose = new Pose(49.5,-20*apm,apm*(Math.PI/-2.0));
            PathChain center_path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, next_pose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), next_pose.getHeading(),0.5)
                    .addParametricCallback(1,()-> follower.setMaxPower(1))
                    .addPath(new BezierLine(next_pose, avoid_gate_pose))
//                    .addParametricCallback(1, () -> { // Pause 80% through the *previous* path
//                        follower.pausePathFollowing(); // Stop robot movement
//                        resume_time = runtime.seconds()+(double) 0.3;
//                        //0.3 is how long the robot will wait to ensure it intakes all artifacts. 0.3 is 1/100 of the time aloted for auto.
//                    })
                    .build();
            follower.setMaxPower(drivetrain_pickup_speed);
            follower.followPath(center_path);
            step = 8;
        } else if (step == 8 && !follower.isBusy()){
//            spin_intake = false;
            follower.setMaxPower(1);
            fired_artifacts = 0;
            intake_round += 1;
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
            Pose next_pose = new Pose(28,-6*apm,apm*(Math.PI/-2.0));
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
            intake_round += 1;
            step = 0;
        } else if (step == 100){

        }
    }
}
