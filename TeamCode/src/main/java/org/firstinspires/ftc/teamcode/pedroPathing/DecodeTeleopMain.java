package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DecodeTeleopMain")

public class DecodeTeleopMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialise motor
    // variables
    private Servo launchKickServo;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotor launchMotor;

    private DcMotor intakeMotor;
    //pedro
    private PathChain path;
    public static Follower follower;
    public static PoseTracker pose_tracker;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        launchKickServo = hardwareMap.get(Servo.class,"launchKickServo");

        launchMotor = hardwareMap.get(DcMotor.class,"LaunchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");

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
        pose_tracker = follower.getPoseTracker();


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

    Pose remembered_pose;

    boolean spin_launcher = false;
    boolean spin_intake = false;

    @Override
    public void loop() {

        launchKickServo.setPosition(gamepad2.left_stick_y);
        telemetry.addData("Servo Angle: ",gamepad2.left_stick_y);

//        if (gamepad2.x) {
//            launchKickServo.setPosition(0.75);
//        } else {
//            launchKickServo.setPosition(1);
//        }
            //way to troubleshoot servo positions

        if (gamepad2.left_stick_x != 0){
            telemetry.addData("servo position",gamepad2.left_stick_x);
            launchKickServo.setPosition(gamepad2.left_stick_x);
        }


        if (gamepad2.yWasPressed()){
            spin_launcher = !spin_launcher;
        }

        if (spin_launcher){
            launchMotor.setPower(1);
        } else {
            launchMotor.setPower(0);
        }

        if (gamepad2.aWasPressed()){
            spin_intake = !spin_intake;
        }

        if (spin_intake){
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }


        if (gamepad1.aWasPressed()){
            pose_tracker.update();
            remembered_pose = follower.getPose();
        }

        if (gamepad1.bWasPressed()){
            if (remembered_pose != null){
                pose_tracker.update();
                Pose current_pose = follower.getPose();
                path = follower.pathBuilder()
                        .addPath(new BezierLine(current_pose, remembered_pose))
//                .addPath(new BezierLine(endPose, nextendPose))
                        .setLinearHeadingInterpolation(current_pose.getHeading(), remembered_pose.getHeading())
                        .build();
                follower.followPath(path);
            }
        }

        if (gamepad1.xWasPressed()){
            pose_tracker.update();
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