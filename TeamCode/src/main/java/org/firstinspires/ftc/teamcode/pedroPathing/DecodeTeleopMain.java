package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DecodeTeleopMain")

public class DecodeTeleopMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialise motor
    // variables
    private Servo launchKickServo1;
    private Servo launchKickServo2;

    private double LaunchServoAngle = 0.35;

    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotorEx launchMotor;

    private DcMotor intakeMotor;
    //pedro
    private PathChain path;
    public static Follower follower;
    public static PoseTracker pose_tracker;

    private int launcherSpeed = 900;
    //ticks per second



    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        launchKickServo1 = hardwareMap.get(Servo.class,"lks1");
        launchKickServo2 = hardwareMap.get(Servo.class,"lks2");

        launchMotor = hardwareMap.get(DcMotorEx.class,"LaunchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //servo start position
        launchKickServo1.setPosition(0);
        launchKickServo2.setPosition(1);

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
        timeSinceShot.reset();

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

    //the initial remembered pose
    private Pose remembered_pose = new Pose(0,0,Math.toRadians(0));

    boolean spin_launcher = true;
    boolean spin_intake = true;

    boolean kick = false;
    private ElapsedTime timeSinceShot = new ElapsedTime();

    private int maxLauncherSpeed = 2200;
    private int minLauncherSpeed = 600;

    @Override
    public void loop() {

//        telemetry.addData("servo position",gamepad2.left_stick_x);
//        launchKickServo1.setPosition(gamepad2.left_stick_x);
//        launchKickServo2.setPosition(1-gamepad2.left_stick_y);

        if ((gamepad2.right_trigger>0.1)||(gamepad1.right_trigger>0.1)) {
            spin_launcher = true;
           if (launchMotor.getVelocity() >= (launcherSpeed-10)){
               if (timeSinceShot.seconds() > 1.5){
                   kick = true;
                   timeSinceShot.reset();
               }
           }
        } else {
            spin_launcher = false;
        }

        if (timeSinceShot.seconds() > 0.5) {
            kick = false;
        }

        if (kick) {
            launchKickServo1.setPosition(LaunchServoAngle);
            launchKickServo2.setPosition(1-LaunchServoAngle);
        } else {
            launchKickServo1.setPosition(0);
            launchKickServo2.setPosition(1);
        }

//        if (gamepad2.yWasPressed()){
//            spin_launcher = !spin_launcher;
//        }

        //range change code

        //replace the max and mins of 1000 and 600 with variablez later

        //Still need a way to visually show this besides telemetry
        if (gamepad2.dpadUpWasPressed()||gamepad1.dpadUpWasPressed()){
            launcherSpeed += 50;
            launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
        } else if (gamepad2.dpadDownWasPressed()||gamepad1.dpadDownWasPressed()) {
            launcherSpeed -= 50;
            launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
        }

        if (spin_launcher){
            launchMotor.setVelocity(launcherSpeed); //ticks/s
        } else {
            launchMotor.setPower(0.05);
        }
        telemetry.addData("launchmotor targetv", launcherSpeed);
        telemetry.addData("launchmotor velocity",launchMotor.getVelocity());//ticks/s

        if (gamepad2.aWasPressed()||gamepad1.rightBumperWasPressed()){
            spin_intake = !spin_intake;
        }

        if (spin_intake){
            intakeMotor.setPower(1);
        } else if(gamepad2.a||gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.aWasPressed()){
            pose_tracker.update();
            remembered_pose = follower.getPose();
        }
        telemetry.addData("remembered pose information","");
        telemetry.addData("heading",remembered_pose.getHeading());
        telemetry.addData("x",remembered_pose.getX());
        telemetry.addData("y",remembered_pose.getY());

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

        if (follower.isBusy()) {
            follower.update();
            follower_was_just_busy = true;
            if (gamepad1.x){
                follower.breakFollowing();
            }
        } else {
            if (follower_was_just_busy == true){
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            follower_was_just_busy = false;
            //manual control for drive, will use user input if pedro is not executing a task.

            double forward = gamepad1.left_stick_y;
            double Strafe = -gamepad1.left_stick_x;
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

    double integralSum = 0;
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0; //feedforward..??
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;

    public double PIDControl(double reference,double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivitive = (error - lastError);
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivitive * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}