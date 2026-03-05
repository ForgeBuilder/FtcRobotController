package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaTurretAutoTest")
public class BallistaTurretAutoTest extends CrossbowMain {

    PathChain test_path;

    private Pose starter_pose = new Pose(-47.515,-58.716,-Math.PI);

    private Pose launch_pose = new Pose(-14.3,-16,-Math.PI);

    private Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
        follower.setStartingPose(starter_pose);
    }

    @Override public void start(){
        super.start();
        PathChain to_first_launch = follower.pathBuilder()
                .addPath(new BezierLine(starter_pose,launch_pose))
                .setConstantHeadingInterpolation(-Math.PI)
                .build();
        follower.followPath(to_first_launch);
    }

    @Override public void loop(){
        super.loop();
        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }

}
