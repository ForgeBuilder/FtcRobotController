package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallistaAutoFar extends BallistaAuto {
    PathChain test_path;
    Pose starter_pose;

    ElapsedTime step_timer = new ElapsedTime();

    Pose launch_2_pose;

    Pose human_zone_corner_pose;
//    Pose launch_pose = new Pose(-14.3,-16,Math.PI);

//    Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
    }


    public AutoStep current_auto_step = AutoStep.None;
    enum AutoStep {
        None, FireFirstVolley, HumanZoneIntakeOne, ReturnToFarLaunchOne,FireSecondVolley,IntakeZoneOne
    }

    @Override public void start(){
        set_launcher_speed(1480);
        super.start();

        starter_pose = new Pose(68.2,-7.1*apm,apm*-Math.PI/2);
        launch_2_pose = new Pose(65.5,-10*apm,apm*-Math.PI/2);

        //CHANGE IT BACK TO 60 OR AS HIGH AS WE NEED TO GET IT TO HIT THE WALL BUT COME BACK
        human_zone_corner_pose = new Pose(66,-61*apm,apm*-Math.PI/2);

        follower.setPose(starter_pose);

        set_step(AutoStep.FireFirstVolley);
        //for red, mod constant is still 1 as long as you spesify 3.14 as the heading shift within the config settings.



//        follower.setStartingPose(starter_pose);
//        PathChain to_first_launch = follower.pathBuilder()
//                .addPath(new BezierLine(starter_pose,test_pose))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//        follower.followPath(to_first_launch);
    }

    @Override public void loop(){
        super.loop();
        turret.update();
        launcher_code(fire_artifact,false);

        panelsTelemetry.addData("step: ",current_auto_step);
        telemetry.addData("step: ",current_auto_step);

        switch (current_auto_step){
            case FireFirstVolley:
                if (open_door && time_since_ball_ready.seconds() > 1.0){
                    turret_stop_firing();
                    fire_artifact = false;
                    set_step(AutoStep.HumanZoneIntakeOne);
                }
                break;
            case HumanZoneIntakeOne:
//                check to see if this if is what is sending us back
                if (ball_ready || !follower.isBusy() || (step_timer.seconds() > 5)) { //(Math.abs(current_pedro_pose.getY()) > 30.0)
                    follower.breakFollowing();
                    follower.setMaxPower(1);
                    set_step(AutoStep.ReturnToFarLaunchOne);
                }
                break;
            case ReturnToFarLaunchOne:
                if (!follower.isBusy() && (Math.abs(follower.getPose().getY())<20)) {
                    set_step(AutoStep.FireSecondVolley);
                }
                break;
            case FireSecondVolley:
                if (open_door && (time_since_ball_ready.seconds() > 1.0)){
                    turret_stop_firing();
                    fire_artifact = false;
                    set_step(AutoStep.HumanZoneIntakeOne);
                }
                break;
        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }
    public void set_step(AutoStep step){

        switch (step){
            case FireFirstVolley:
                time_since_ball_ready.reset();
                follower.holdPoint(starter_pose);
                spin_launcher = true;
                fire_artifact = true;
                break;
            case HumanZoneIntakeOne:
                //CHANGE THIS BACK TO 1 LATER
                //REMOVE, IMPORTANT DADADA SEE ME KEYWORDS
                follower.setMaxPower(1);

                follower.breakFollowing();
                PathChain human_zone_intake_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),human_zone_corner_pose))
                        .setConstantHeadingInterpolation(human_zone_corner_pose.getHeading())
                        .addParametricCallback(0.4, () ->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(human_zone_intake_path);
                step_timer.reset();
                break;
            case ReturnToFarLaunchOne:
                follower.setMaxPower(1);
                PathChain return_to_launch_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),launch_2_pose))
                        .setConstantHeadingInterpolation(launch_2_pose.getHeading())
                        .addParametricCallback(0.2, () ->{
                            spin_intake = false;
                        })
                        .addParametricCallback(0.4, () ->{
                            follower.setMaxPower(0.2);
                        })
                        .build();
                follower.followPath(return_to_launch_path);
                break;
            case FireSecondVolley:
                follower.setMaxPower(1);
                time_since_ball_ready.reset();
                spin_intake = false;
                follower.holdPoint(launch_2_pose);
                spin_launcher = true;
                fire_artifact = true;
                break;
        }
        current_auto_step = step;
    }
}
