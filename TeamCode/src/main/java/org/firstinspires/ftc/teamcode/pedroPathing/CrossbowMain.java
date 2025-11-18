package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision; //ah you can't do this
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//@TeleOp(name="DecodeTeleopMain")


public class CrossbowMain extends OpMode {
    // Declare OpMode members.
    private Servo launchKickServo1;
    private Servo launchKickServo2;

    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotorEx rightLaunchMotor;
    private DcMotorEx leftLaunchMotor;

    private DcMotorEx intakeMotor;

    Limelight3A limelight;

    //universal pedro stuff

    public static Follower follower;
    public static PoseTracker pose_tracker;
//    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(0,0,0,0);
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

        rightLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm1");
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);

        leftLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm2");
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

        telemetry.addData("Status", "Initialized");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

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
        timeSinceShot.reset();
    }

    @Override
    public void loop() {
    }


    public void follower_code(boolean break_path){
        follower.update();
        follower_was_just_busy = true;
        if (break_path){
            follower.breakFollowing();
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private boolean follower_was_just_busy = true;

    //the initial remembered pose


    boolean spin_launcher = true;

    boolean kick = false;
    private ElapsedTime timeSinceShot = new ElapsedTime();

    private int maxLauncherSpeed = 2200;
    private int minLauncherSpeed = 600;

    //for telemetry - I should really start breaking this stuff into functions so I can init variables near where they are used.
    double left_speed_at_kick = 0.0;
    double right_speed_at_kick = 0.0;

    private double KickerLaunchAngle = 0.35;
    private double KickerIdleAngle = 0;

    private int launcherSpeed = 900;
    //ticks per second
    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(200,2,0,0);


    //returns true each time it fires the artifact. indicates when the robot has decided to fire, not when the shot is clear.
    //do not move the instant this function returns true. You may attempt to fire again.
    public boolean launcher_code(boolean fire){
        //the return value of the function: did the robot fire the artifact
        boolean fired_this_tick = false;
        if (fire) {
            spin_launcher = true;
            //when the motor's velocity is equal - so when it fires, ball will likley be slightly overshot.

            boolean right_speed_met = rightLaunchMotor.getVelocity() == launcherSpeed;
            boolean left_speed_met = leftLaunchMotor.getVelocity() == launcherSpeed;

            if ((right_speed_met && left_speed_met) || gamepad1.right_bumper){//the right bumper serves as an override
                if (timeSinceShot.seconds() > 1.5){
                    kick = true;
                    timeSinceShot.reset();
                    //debug information - motor 2 is left, motor 1 is right
                    //
                    left_speed_at_kick = leftLaunchMotor.getVelocity();
                    right_speed_at_kick = rightLaunchMotor.getVelocity();
                    fired_this_tick = true;
                }
            }
        } else {
            spin_launcher = false;
            fired_this_tick = false;
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
            rightLaunchMotor.setPower(1);
            rightLaunchMotor.setVelocity(launcherSpeed); //ticks/s
            leftLaunchMotor.setPower(1);
            leftLaunchMotor.setVelocity(-1*launcherSpeed); //ticks/s
        } else {

            rightLaunchMotor.setPower(0);
            leftLaunchMotor.setPower(0);
        }
        telemetry.addData("launchmotor targetv", launcherSpeed);
        telemetry.addData("launchmotor1 velocity", rightLaunchMotor.getVelocity());//ticks/s
        telemetry.addData("launchmotor2 velocity", leftLaunchMotor.getVelocity());//ticks/s

        return fired_this_tick;
    }

    LLResult LLresult;
    public void teleop_limelight_code(){
        //limelight stuff
        LLresult = limelight.getLatestResult();
        if (LLresult != null && LLresult.isValid()) {
            double tx = LLresult.getTx(); // How far left or right the target is (degrees)
            double ty = LLresult.getTy(); // How far up or down the target is (degrees)
            double ta = LLresult.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
//            telemetry.addData("Target Y", ty);
//            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    boolean spin_intake = false;
    public void intake_code(){
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
    }

    private Pose teleop_remembered_pose = new Pose(0,0,Math.toRadians(0));
    //run every tick with no arguments for ability to save and return to position in teleop
    public void teleop_return_to_position(){
        if (gamepad1.aWasPressed()){
            pose_tracker.update();
            teleop_remembered_pose = follower.getPose();
        }
        telemetry.addData("remembered pose information","");
        telemetry.addData("heading",teleop_remembered_pose.getHeading());
        telemetry.addData("x",teleop_remembered_pose.getX());
        telemetry.addData("y",teleop_remembered_pose.getY());

        if (gamepad1.bWasPressed()){
            if (teleop_remembered_pose != null){
                pose_tracker.update();
                Pose current_pose = follower.getPose();
                PathChain path = follower.pathBuilder()
                        .addPath(new BezierLine(current_pose, teleop_remembered_pose))
                        .setLinearHeadingInterpolation(current_pose.getHeading(), teleop_remembered_pose.getHeading())
                        .build();
                follower.followPath(path);
            }
        }
    }

    public void drive_with_teleop(){
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
            turn+= 0.03*LLresult.getTx();
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}