package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="PeterPedroAutoTest",preselectTeleOp = "TeleopMain")



public class PeterPedroAutoTest extends OpMode {
    public static Follower follower;
    private PathChain squareLoop;
    private final Pose startPose = new Pose(0,0,0);
    private final Pose midPose = new Pose(0,10,0);
    private final Pose jeffPose = new Pose(10,10,0);
    private final Pose jeffyPose = new Pose(10,0,0);

    private boolean looping = false;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        follower.activateDrive();

        squareLoop = follower.pathBuilder()
                .addPath(new BezierLine(startPose,midPose))
                .addPath(new BezierLine(midPose,jeffPose))
                .addPath(new BezierLine(jeffPose, jeffyPose))
                .addPath(new BezierLine(jeffyPose, startPose))
                .setLinearHeadingInterpolation(0,0)
                .build();
        follower.followPath(squareLoop);
        looping = true;
    }

    @Override
    public void loop() {
        follower.update();
        if (looping && !follower.isBusy())
            follower.followPath(squareLoop);
    }
}