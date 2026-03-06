package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaAutoNear")
public class BallistaAutoNear extends BallistaAuto {

    Pose starter_pose = new Pose(-45.2, -60.8, -Math.PI / 2);

    Pose launch_one_pose = new Pose(-30, -35, -Math.PI / 2);


    Pose intake_near_bar_pose_1 = new Pose(-20,-24,-Math.PI / 2);
    Pose intake_near_bar_pose_2 = new Pose(-20,-60,-Math.PI / 2);

    Pose intake_mid_bar_pose_1 = new Pose(14,-24,-Math.PI / 2);
    Pose intake_mid_bar_pose_2 = new Pose(14,-60,-Math.PI / 2);

    Pose intake_far_bar_pose_1 = new Pose(28,-24,-Math.PI / 2);
    Pose intake_far_bar_pose_2 = new Pose(28,-60,-Math.PI / 2);


//    PathConstraints path_constraint = new PathConstraints()

    @Override
    public void init() {
        super.init();
        set_team("blue");
        follower.setPose(starter_pose);
    }


    public AutoStep current_auto_step = AutoStep.None;

    enum AutoStep {
        None,
        ReturnToLaunchNearOne,
        FireFirstVolley,
        IntakeNearBar,
        ReturnToLaunchNearTwo,
        FireSecondVolley,
        IntakeMidBar,
        ReturnToLaunchNearThree,
        FireThirdVolley,
        IntakeFarBar,
        ReturnToLaunchNearFour,
        FireFourthVolley
    }

    @Override
    public void start() {
        super.start();
        spin_launcher = true;
        set_step(AutoStep.ReturnToLaunchNearOne);
        //for red, mod constant is still 1 as long as you spesify 3.14 as the heading shift within the config settings.


//        follower.setStartingPose(starter_pose);
//        PathChain to_first_launch = follower.pathBuilder()
//                .addPath(new BezierLine(starter_pose,test_pose))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//        follower.followPath(to_first_launch);
    }

    @Override
    public void loop() {
        super.loop();
        turret.update();
        launcher_code(fire_artifact, false);

        panelsTelemetry.addData("step: ", current_auto_step);
        telemetry.addData("step: ", current_auto_step);

        switch (current_auto_step) {
            case ReturnToLaunchNearOne:
                if (!follower.isBusy()){
                    set_step(AutoStep.FireFirstVolley);
                }
                break;
            case FireFirstVolley:
                if (turret_fire_loop() == turret_firing_state.FINISHED_FIRING){
                    turret_stop_firing();
                    set_step(AutoStep.IntakeNearBar);
                }
                break;
            case IntakeNearBar:
                if ((get_balls_loaded_state() == balls_loaded_state.FULL) || !follower.isBusy()){
                    set_step(AutoStep.ReturnToLaunchNearTwo);
                }
            case ReturnToLaunchNearTwo:
                if (!follower.isBusy()){
                    set_step(AutoStep.FireSecondVolley);
                }
                break;
            case FireSecondVolley:
                if (turret_fire_loop() == turret_firing_state.FINISHED_FIRING){
                    turret_stop_firing();
                    set_step(AutoStep.IntakeMidBar);
                }
                break;
            case IntakeMidBar:
                if ((get_balls_loaded_state() == balls_loaded_state.FULL) || !follower.isBusy()){
                    set_step(AutoStep.ReturnToLaunchNearThree);
                }
                break;
            case ReturnToLaunchNearThree:
                if (!follower.isBusy()){
                    set_step(AutoStep.FireThirdVolley);
                }
                break;
            case FireThirdVolley:
                if (turret_fire_loop() == turret_firing_state.FINISHED_FIRING){
                    turret_stop_firing();
                    set_step(AutoStep.IntakeFarBar);
                }
                break;
            case IntakeFarBar:
                if ((get_balls_loaded_state() == balls_loaded_state.FULL) || !follower.isBusy()){
                    set_step(AutoStep.ReturnToLaunchNearFour);
                }
                break;
            case ReturnToLaunchNearFour:
                if (!follower.isBusy()){
                    set_step(AutoStep.FireFourthVolley);
                }
                break;
        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }

    public void set_step(AutoStep step) {

        switch (step) {
            case ReturnToLaunchNearOne:
                PathChain return_to_near_one = follower.pathBuilder()
                        .addPath(new BezierLine(current_pedro_pose,launch_one_pose))
                        .setConstantHeadingInterpolation(launch_one_pose.getHeading())
                        .addParametricCallback(0.5,()->{
                            spin_intake = false;
                        })
                        .build();
                follower.followPath(return_to_near_one);
                break;
            case FireFirstVolley:
                start_turret_fire(starter_pose);
                break;
            case IntakeNearBar:
                PathChain to_intake_bar_one = follower.pathBuilder()
                        .addPath(new BezierLine(launch_one_pose, intake_near_bar_pose_1))
                        .setConstantHeadingInterpolation(intake_near_bar_pose_1.getHeading())
                        .addPath(new BezierLine(intake_near_bar_pose_1, intake_near_bar_pose_2))
                        .setConstantHeadingInterpolation(intake_near_bar_pose_2.getHeading())
//                        .addPath(new BezierCurve(launch_one_pose, intake_far_bar_pose_1,intake_far_bar_pose_1))
//                        .setConstantHeadingInterpolation(intake_far_bar_pose_1.getHeading())
                        .addParametricCallback(0.2,()->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(to_intake_bar_one);
                break;
            case ReturnToLaunchNearTwo:
                PathChain return_to_near_two = follower.pathBuilder()
                        .addPath(new BezierLine(current_pedro_pose, launch_one_pose))
                        .setConstantHeadingInterpolation(launch_one_pose.getHeading())
                        .addParametricCallback(0.5,()->{
                            spin_intake = false;
                        })
                        .build();
                follower.followPath(return_to_near_two);
                break;
            case FireSecondVolley:
                start_turret_fire(launch_one_pose);
                break;
            case IntakeMidBar:
                PathChain intake_mid_bar_path = follower.pathBuilder()
                        .addPath(new BezierLine(current_pedro_pose, intake_near_bar_pose_1))
                        .setConstantHeadingInterpolation(intake_near_bar_pose_1.getHeading())
                        .addPath(new BezierLine(intake_near_bar_pose_1, intake_near_bar_pose_2))
                        .setConstantHeadingInterpolation(intake_near_bar_pose_2.getHeading())
                        .addParametricCallback(0.2,()->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(intake_mid_bar_path);
                break;
            case ReturnToLaunchNearThree:
                PathChain return_to_near_three = follower.pathBuilder()
                        .addPath(new BezierLine(current_pedro_pose, launch_one_pose))
                        .setConstantHeadingInterpolation(launch_one_pose.getHeading())
                        .addParametricCallback(0.5,()->{
                            spin_intake = false;
                        })
                        .build();
                follower.followPath(return_to_near_three);
                break;
            case FireThirdVolley:
                start_turret_fire(launch_one_pose);
                break;
            case IntakeFarBar:
                PathChain intake_far_bar_path = follower.pathBuilder()
                        .addPath(new BezierLine(current_pedro_pose, intake_far_bar_pose_1))
                        .setConstantHeadingInterpolation(intake_far_bar_pose_2.getHeading())
                        .addPath(new BezierLine(intake_far_bar_pose_1, intake_far_bar_pose_2))
                        .setConstantHeadingInterpolation(intake_far_bar_pose_2.getHeading())
                        .addParametricCallback(0.2,()->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(intake_far_bar_path);
                break;
            case ReturnToLaunchNearFour:
                break;
            case FireFourthVolley:
                start_turret_fire(launch_one_pose);
                break;
        }
        current_auto_step = step;
    }
}
