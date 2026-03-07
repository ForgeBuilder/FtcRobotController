package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaTurretAutoTest")
public class BallistaTurretAutoTest extends BallistaMain {

    PathChain test_path;

    Pose starter_pose = new Pose(-47.515,-58.716,-Math.PI);

    Pose test_pose = new Pose(-42,-55,-Math.PI);
    Pose launch_pose = new Pose(-14.3,-16,-Math.PI);

    Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
    }

    @Override public void start(){
        super.start();
        follower.setStartingPose(starter_pose);
        PathChain to_first_launch = follower.pathBuilder()
                .addPath(new BezierLine(starter_pose,test_pose))
                .setConstantHeadingInterpolation(-Math.PI)
                .build();
        follower.followPath(to_first_launch);
    }

    @Override public void loop(){
        super.loop();
//        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }

}
