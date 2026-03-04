package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaPedroUnitTest")
public class BallistaPedroUnitTest extends CrossbowMain {
    @Override public void start(){
        super.start();

        Pose pose_a = new Pose(0,0,0);

        Pose pose_b = new Pose(10,0,Math.PI);

        Pose pose_c = new Pose(10,10,Math.PI);

        Pose pose_d = new Pose(0,10,0);

        PathChain test_path = follower.pathBuilder()
                .addPath(new BezierLine(pose_a, pose_b))
                .setLinearHeadingInterpolation(pose_a.getHeading(), pose_b.getHeading(), 0.5)
                .addPath(new BezierLine(pose_b, pose_c))
                .setConstantHeadingInterpolation(Math.PI)
                .addPath(new BezierLine(pose_c, pose_d))
                .setLinearHeadingInterpolation(pose_c.getHeading(), pose_d.getHeading(), 0.5)
                .addPath(new BezierLine(pose_d, pose_a))
                .setConstantHeadingInterpolation(0)
                .build();
        follower.followPath(test_path);
    }
}
