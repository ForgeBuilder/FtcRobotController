package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="BlueAuto1",preselectTeleOp = "TeleopMain")



public class BlueAuto1 extends OpMode {

    public static double DISTANCE = 40;
    private boolean forward = true;
    public static Follower follower;
    private Path forwards;
    private Path backwards;

    private PathChain path;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
    private final Pose endPose = new Pose(20, 0, Math.toRadians(0)); // this is a way to define a pose
//    private final Pose midPose = new Pose(15, 20, Math.toRadians(0));
//    private final Pose nextendPose = new Pose(20, 0, Math.toRadians(3.14));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    public void start() {
        follower.activateDrive();

        //becasue we start with the intake facing backwards and the intake faces forwards, we start backwards. thus this is vital.
        follower.setPose(new Pose(0,0,Math.toRadians(180)));

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        follower.followPath(path);
    }

    @Override
    public void loop() {
        if (follower.isBusy()){
            follower.update();
        }
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    }