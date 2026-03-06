package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaFarTriangleRNDTest")
public class BallistaFarTriangleRNDTest extends CrossbowMain {

    boolean fire_artifact;
    PathChain test_path;

    Pose starter_pose = new Pose(68.2,7.1,Math.PI/2);

    Pose human_zone_corner_pose = new Pose(66,60,Math.PI/2);
//    Pose launch_pose = new Pose(-14.3,-16,Math.PI);

//    Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
        set_team("red");
        follower.setPose(starter_pose);
    }


    public AutoStep current_auto_step = AutoStep.FireFirstVolley;
    enum AutoStep {
        FireFirstVolley, HumanZoneIntakeOne, ReturnToFarLaunchOne,FireSecondVolley,IntakeZoneOne
    }

    @Override public void start(){
        super.start();
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
                if (open_door && (time_since_ball_ready.seconds() > 1.0)){
                    follower.breakFollowing();
                    fire_artifact = false;
                    set_step(AutoStep.HumanZoneIntakeOne);
                }
                break;
            case HumanZoneIntakeOne:
                if (!follower.isBusy())
                    set_step(AutoStep.ReturnToFarLaunchOne);
                break;
            case ReturnToFarLaunchOne:
                if (!follower.isBusy())
                    set_step(AutoStep.FireSecondVolley);
                break;
            case FireSecondVolley:
                if (open_door && (time_since_ball_ready.seconds() > 1.0)){
                    follower.breakFollowing();
                    fire_artifact = false;
                    set_step(AutoStep.IntakeZoneOne);
                }
                break;
        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }
    public void set_step(AutoStep step){

        switch (step){
            case FireFirstVolley:
                follower.holdPoint(starter_pose);
                spin_launcher = true;
                fire_artifact = true;
                current_auto_step = AutoStep.FireFirstVolley;
                break;
            case HumanZoneIntakeOne:
                PathChain zone_one_intake_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),human_zone_corner_pose))
                        .setConstantHeadingInterpolation(human_zone_corner_pose.getHeading())
                        .addParametricCallback(0.4, () ->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(zone_one_intake_path);
                current_auto_step = AutoStep.HumanZoneIntakeOne;
                break;
            case ReturnToFarLaunchOne:
                Pose launch_2_pose = new Pose(68.2,14,Math.PI/2);
                PathChain return_to_launch_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),starter_pose))
                        .setConstantHeadingInterpolation(starter_pose.getHeading())
                        .addParametricCallback(0.3, () ->{
                            spin_intake = false;
                            follower.setMaxPower(0.4);
                        })
                        .build();
                follower.followPath(return_to_launch_path);
                current_auto_step = AutoStep.ReturnToFarLaunchOne;
                break;
            case FireSecondVolley:
                follower.holdPoint(starter_pose);
                spin_launcher = true;
                fire_artifact = true;
                current_auto_step = AutoStep.FireSecondVolley;
                break;
        }
    }
}
