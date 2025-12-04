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
    private final Pose startPose = new Pose(0,0,0); //start at ze0ro
    private final Pose midPose = new Pose(0,10,0); // move ten inches up
    private final Pose jeffPose = new Pose(10,10,0); // move ten inches right
    private final Pose jeffyPose = new Pose(10,0,0); // move then inches down

    private boolean looping = false; // allows the robor to build the path first and then just loop when this is tru
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // start pose becomes in fact start pose
    }

    @Override
    public void start() {
        follower.activateDrive();
        // move the robot and then keep the angle the same the whole time
        squareLoop = follower.pathBuilder()
                .addPath(new BezierLine(startPose,midPose))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(midPose,jeffPose))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(jeffPose, jeffyPose))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(jeffyPose, startPose))
                .setConstantHeadingInterpolation(0)
                .build();
        follower.followPath(squareLoop);
        looping = true;
    }

    @Override
    public void loop() {
        follower.update();
        // loop that robot, only happen after first path and when finished with the loop
        if (looping && !follower.isBusy())
            follower.followPath(squareLoop);
    }
}