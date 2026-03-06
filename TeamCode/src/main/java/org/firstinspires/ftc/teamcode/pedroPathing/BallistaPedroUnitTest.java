package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaPedroUnitTest")
public class BallistaPedroUnitTest extends CrossbowMain {


    Pose start_pose = new Pose (0,0,0);
    Pose pose_a = new Pose(5,5,0);

    Pose pose_b = new Pose(5,-5,Math.PI/4);

    Pose pose_c = new Pose(-5,-5,Math.PI/4);

    Pose pose_d = new Pose(-5,5,0);

    PathChain test_path;

    @Override public void start(){
        super.start();

        test_path = follower.pathBuilder()
                .addPath(new BezierLine(pose_a, pose_b))
                .setLinearHeadingInterpolation(pose_a.getHeading(), pose_b.getHeading(), 0.5)
                .addPath(new BezierLine(pose_b, pose_c))
                .setConstantHeadingInterpolation(pose_c.getHeading())
                .addPath(new BezierLine(pose_c, pose_d))
                .setLinearHeadingInterpolation(pose_c.getHeading(), pose_d.getHeading(), 0.5)
                .addPath(new BezierLine(pose_d, pose_a))
                .setConstantHeadingInterpolation(pose_d.getHeading())
                .build();
        follower.followPath(test_path);
    }



    @Override public void loop(){
        super.loop();
        if (!follower.isBusy()){
            follower.setMaxPower(0.3);
            test_path = follower.pathBuilder()
                    .addPath(new BezierLine(pose_a, pose_b))
                    .setLinearHeadingInterpolation(pose_a.getHeading(), pose_b.getHeading(), 0.5)
                    .addPath(new BezierLine(pose_b, pose_c))
                    .setConstantHeadingInterpolation(pose_c.getHeading())
                    .addPath(new BezierLine(pose_c, pose_d))
                    .setLinearHeadingInterpolation(pose_c.getHeading(), pose_d.getHeading(), 0.5)
                    .addPath(new BezierLine(pose_d, pose_a))
                    .setConstantHeadingInterpolation(pose_d.getHeading())
                    .build();
            follower.followPath(test_path);
        }
        panelsTelemetry.update(telemetry);
    }
}
