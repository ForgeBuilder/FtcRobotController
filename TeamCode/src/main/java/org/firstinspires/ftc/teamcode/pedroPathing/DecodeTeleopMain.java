package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DecodeTeleopMain")

public class DecodeTeleopMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialise motor
    // variables
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    //pedro
    private PathChain path;
    public static Follower follower;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        telemetry.addData("Status", "Initialized");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        //Make every motor break when at power 0

        // make the right side wheels reversed
        // leftBack.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //pedro
        follower = Constants.createFollower(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

//        follower.activateDrive();

        final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
        final Pose endPose = new Pose(10, 2, Math.toRadians(0)); // this is a way to define a pose
        final Pose nextendPose = new Pose(20, 0, Math.toRadians(90));

//        path = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, endPose))
////                .addPath(new BezierLine(endPose, nextendPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), nextendPose.getHeading())
//                .build();
//        follower.followPath(path);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private boolean follower_was_just_busy = true;
    final Pose endPose = new Pose(10, 2, Math.toRadians(0)); // this is a way to define a pose
    final Pose nextendPose = new Pose(20, 0, Math.toRadians(90));

    final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose

    @Override
    public void loop() {

        if (gamepad1.xWasPressed()){
            follower.update();
            Pose current_pose = follower.getPose();
            path = follower.pathBuilder()
                    .addPath(new BezierLine(current_pose, endPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                    .setLinearHeadingInterpolation(current_pose.getHeading(), nextendPose.getHeading())
                    .build();
            follower.followPath(path);
        }


        if (follower.isBusy()) {
            follower.update();
            follower_was_just_busy = true;
        } else {
            if (follower_was_just_busy == true){
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            follower_was_just_busy = false;
            //manual control for drive, will use user input if pedro is not executing a task.

            double forward = -gamepad1.left_stick_y;
            double Strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //slide recalibrate..

            //power slides so they retract and stop/reset encoders with the top bumpers

            //driver 1's slowmode

            double slowdown = 1 - (gamepad1.left_trigger * .75);


            forward = forward * slowdown;
            Strafe = Strafe * slowdown;
            turn = turn * slowdown;

            //thiz iz giving me null pointer exzecptionz for zome reazon vvv it zayz the referencez to the motorz are null objectz.. what??

            //setpower for drive

            leftFront.setPower(forward + Strafe + turn);
            leftBack.setPower(forward - (Strafe - turn));
            rightFront.setPower(forward - (Strafe + turn));
            rightBack.setPower(forward + (Strafe - turn));
        }



        // Show the elapsed game time and update telemetry so we can see it
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}