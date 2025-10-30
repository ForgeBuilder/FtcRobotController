package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.paths.*;

@Autonomous(name="PedroAutoTest",preselectTeleOp = "TeleopMain")



public class PedroAutoTest extends OpMode {

    public static double DISTANCE = 40;
    private boolean forward = true;
    public static Follower follower;
    private Path forwards;
    private Path backwards;

    private PathChain path;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
    private final Pose endPose = new Pose(10, 2, Math.toRadians(0)); // this is a way to define a pose
    private final Pose midPose = new Pose(15, 20, Math.toRadians(0));
    private final Pose nextendPose = new Pose(20, 0, Math.toRadians(3.14));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    public void start() {
        follower.activateDrive();

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .addPath(new BezierLine(midPose, startPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), nextendPose.getHeading())
                .build();
        follower.followPath(path);
    }

    @Override
    public void loop() {
        if (follower.isBusy()){
            follower.update();

        } else {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, midPose))
                    .addPath(new BezierLine(midPose, startPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), nextendPose.getHeading())
                    .build();
            follower.followPath(path);
        }
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    }