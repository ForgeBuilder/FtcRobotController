package org.firstinspires.ftc.teamcode.pedroPathing;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="BasketAutoPedro",preselectTeleOp = "TeleopMain")



public class BasketAutoPedro extends OpMode {

    public static double DISTANCE = 40;
    private boolean forward = true;
    public static Follower follower;
    private Path forwards;
    private Path backwards;

    private PathChain path;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
    private final Pose endPose = new Pose(10, 2, Math.toRadians(3.14)); // this is a way to define a pose

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    public void start() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    }