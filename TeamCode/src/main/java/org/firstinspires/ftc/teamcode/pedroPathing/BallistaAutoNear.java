package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaAutoNear")
public class BallistaAutoNear extends BallistaAuto {

    Pose starter_pose = new Pose(-45.2, -60.8, -Math.PI / 2);

    Pose launch_one_pose = new Pose(-20, -25, -Math.PI / 2);

    Pose intake_far_for_pose = new Pose(-15,-61,-Math.PI / 2);

    @Override
    public void init() {
        super.init();
        set_team("red");
        follower.setPose(starter_pose);
    }


    public AutoStep current_auto_step;

    enum AutoStep {
        ReturnToLaunchNearOne,FireFirstVolley,IntakeFarBar
    }

    @Override
    public void start() {
        super.start();
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
                if (open_door && time_since_ball_ready.seconds() > 1.0){
                    follower.breakFollowing();
                    fire_artifact = false;
                    set_step(AutoStep.IntakeFarBar);
                }
                break;
        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }

    public void set_step(AutoStep step) {

        switch (step) {
            case ReturnToLaunchNearOne:
                PathChain to_launch_one = follower.pathBuilder()
                        .addPath(new BezierLine(starter_pose,launch_one_pose))
                        .setConstantHeadingInterpolation(starter_pose.getHeading())
                        .build();
                break;
            case FireFirstVolley:
                time_since_ball_ready.reset();
                follower.holdPoint(starter_pose);
                spin_launcher = true;
                fire_artifact = true;
                current_auto_step = AutoStep.FireFirstVolley;
                break;
        }
    }
}
