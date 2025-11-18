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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision; //ah you can't do this
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="DecodeTeleopMain")


public class DecodeTeleopMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialise motor
    // variables
    private Servo launchKickServo1;
    private Servo launchKickServo2;

    private double KickerLaunchAngle = 0.35;
    private double KickerIdleAngle = 0;

    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotorEx launchMotor1;
    private DcMotorEx launchMotor2;

    private DcMotorEx intakeMotor;
    //pedro
    private PathChain path;
    public static Follower follower;
    public static PoseTracker pose_tracker;

    private int launcherSpeed = 900;
    //ticks per second

    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(200,2,0,0);
//    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(0,0,0,0);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    Limelight3A limelight;



    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        launchKickServo1 = hardwareMap.get(Servo.class,"lks1");
        launchKickServo2 = hardwareMap.get(Servo.class,"lks2");

        launchMotor1 = hardwareMap.get(DcMotorEx.class,"lm1");
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);

        launchMotor2 = hardwareMap.get(DcMotorEx.class,"lm2");
        launchMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

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

        //limelight camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(9); // Switch to pipeline number 0
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
    boolean spin_intake = false;

    boolean kick = false;
    private ElapsedTime timeSinceShot = new ElapsedTime();

    private int maxLauncherSpeed = 2200;
    private int minLauncherSpeed = 600;

    private int selector = 0;

    private Double[] PIDFCoefficientsList = {200.0,20.0,0.0,0.0};

    //for telemetry - I should really start breaking this stuff into functions so I can init variables near where they are used.
    double left_speed_at_kick = 0.0;
    double right_speed_at_kick = 0.0;

    @Override
    public void loop() {
        //limelight stuff
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
//            telemetry.addData("Target Y", ty);
//            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }



        if (gamepad2.dpadLeftWasPressed()){
            selector+=1;
            if (selector == 4){
                selector = 0;
            }
        }
        if (gamepad2.dpadRightWasPressed()){
            selector-=1;
            if (selector == -1){
                selector = 3;
            }
        }
        if (gamepad2.dpadUpWasPressed()){
            PIDFCoefficientsList[selector] += 1.0;
        }
        if (gamepad2.dpadDownWasPressed()){
            PIDFCoefficientsList[selector] -= 1.0;
        }
        telemetry.addData("p",PIDFCoefficientsList[0]);
        telemetry.addData("i",PIDFCoefficientsList[1]/10);
        telemetry.addData("d",PIDFCoefficientsList[2]);
        telemetry.addData("f",PIDFCoefficientsList[3]);
        if (gamepad1.xWasPressed()){
            launcherCoefficients = new PIDFCoefficients(PIDFCoefficientsList[0],PIDFCoefficientsList[1]/10,PIDFCoefficientsList[2],PIDFCoefficientsList[3]);
            launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
            launchMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
        }

//        telemetry.addData("servo position",gamepad2.left_stick_x);
//        launchKickServo1.setPosition(gamepad2.left_stick_x);
//        launchKickServo2.setPosition(1-gamepad2.left_stick_y);

        if ((gamepad2.right_trigger>0.1)||(gamepad1.right_trigger>0.1)) {
            spin_launcher = true;
            //when the motor's velocity is equal - so when it fires, ball will likley be slightly overshot.
           if (((launchMotor1.getVelocity() == (launcherSpeed))&&(launchMotor1.getVelocity() == (launcherSpeed)))||gamepad1.right_bumper){
               if (timeSinceShot.seconds() > 1.5){
                   kick = true;
                   timeSinceShot.reset();
                   //debug information - motor 2 is left, motor 1 is right
                   //
                   left_speed_at_kick = launchMotor2.getVelocity();
                   right_speed_at_kick = launchMotor2.getVelocity();
               }
           }
        } else {
            spin_launcher = false;
        }
        telemetry.addData("left_speed_at_kick",left_speed_at_kick);
        telemetry.addData("right_speed_at_kick",right_speed_at_kick);

        if (timeSinceShot.seconds() > 0.5) {
            kick = false;
        }

        if (kick) {
            launchKickServo1.setPosition(KickerLaunchAngle);
            launchKickServo2.setPosition(1- KickerLaunchAngle);
        } else {
            launchKickServo1.setPosition(KickerIdleAngle);
            launchKickServo2.setPosition(1-KickerIdleAngle);
        }

//        if (gamepad2.yWasPressed()){
//            spin_launcher = !spin_launcher;
//        }

        //range change code

        //replace the max and mins of 1000 and 600 with variablez later

        //Still need a way to visually show this besides telemetry
        if (gamepad1.dpadUpWasPressed()){//||gamepad1.dpadUpWasPressed()
            launcherSpeed += 40;
            launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
        } else if (gamepad1.dpadDownWasPressed()) {//||gamepad1.dpadDownWasPressed()
            launcherSpeed -= 40;
            launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
        }

        if (spin_launcher){
            launchMotor1.setPower(1);
            launchMotor1.setVelocity(launcherSpeed); //ticks/s
            launchMotor2.setPower(1);
            launchMotor2.setVelocity(-1*launcherSpeed); //ticks/s
        } else {

            launchMotor1.setPower(0);
            launchMotor2.setPower(0);
        }
        telemetry.addData("launchmotor targetv", launcherSpeed);
        telemetry.addData("launchmotor1 velocity", launchMotor1.getVelocity());//ticks/s
        telemetry.addData("launchmotor2 velocity", launchMotor2.getVelocity());//ticks/s

        if (gamepad2.aWasPressed()||gamepad1.leftBumperWasPressed()){
            spin_intake = !spin_intake;
        }   


        //it's a 312 so 537.7 PPR at the Output Shaft. 5.2 RPS (max) would be 2796.04 or about 2800.
        if (spin_intake&&(!kick)){
            intakeMotor.setVelocity(2000);
        } else if(gamepad2.a||gamepad1.left_bumper) {
            intakeMotor.setPower(-2000);
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
            double Strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //if we are trying to fire, line up with the goal.
            if ((gamepad1.right_trigger > 0.1)||gamepad2.right_trigger > 0.1) {
                turn+= 0.03*result.getTx();
            }

            //slide recalibrate..

            //power slides so they retract and stop/reset encoders with the top bumpers

            //driver 1's slowmode

            double slowdown = 1 - (gamepad1.left_trigger * .75);


            forward = forward * slowdown;
            Strafe = Strafe * slowdown;
            turn = turn * slowdown;

            //thiz iz giving me null pointer exzecptionz for zome reazon vvv it zayz the referencez to the motorz are null objectz.. what??

            //setpower for drive

            leftFront.setPower(forward - Strafe + turn);
            leftBack.setPower(forward + Strafe + turn);
            rightFront.setPower(forward + Strafe - turn);
            rightBack.setPower(forward - Strafe - turn);
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