package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.paths.*;

@Autonomous(name="BasketAuto)",preselectTeleOp = "TeleopMain")



public class BasketAutoPedro extends LinearOpMode {

    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    private Path path;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose

    @Override
    public void runOpMode() {
    }
//        path = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//            .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//            .build();
//        follower.followPath(path);
    }