package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Z_BlueAuto1",preselectTeleOp = "DecodeTeleopMain")



public class BlueAuto1 extends OpMode {

    private DcMotor intakeMotor;
    private Servo launchKickServo1;
    private Servo launchKickServo2;
    public static double DISTANCE = 40;
    private boolean forward = true;
    public static Follower follower;
    private Path forwards;
    private Path backwards;

    private DcMotorEx launchMotor;
    private PathChain path;
    private final Pose firstPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
    private final Pose secondPose = new Pose(30, 0, 0); // this should be aiming at the back plate
    private final Pose thirdPose = new Pose(30, 0, Math.toRadians(0));

    //10, 3, 0, 0 is the default for 312s converted to 6000s.
    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(10,3,0.01,0);
//    private PIDFCoefficients currentCoefficients;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        launchKickServo1 = hardwareMap.get(Servo.class,"lks1");
        launchKickServo2 = hardwareMap.get(Servo.class,"lks2");
        launchMotor = hardwareMap.get(DcMotorEx.class,"LaunchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");

        //servo start position
        launchKickServo1.setPosition(0);
        launchKickServo2.setPosition(1);

//        for grabbing the coificcients
//        currentCoefficients = launchMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
    }

    public void start() {
        //spin up the flywheel
        intakeMotor.setPower(1);
        follower.activateDrive();

        //becasue we start with the intake facing backwards and the intake faces forwards, we start backwards. thus this is vital.
//        follower.setPose(new Pose(0,0,Math.toRadians(180)));
    }

    private double LaunchServoAngle = 0.35;
    private boolean kick = false;

    private int fired_count = 0;

    private boolean move1done = false;
    private boolean follower_was_just_busy = false;

    private boolean spin_launcher = false;
    private ElapsedTime timeSinceShot = new ElapsedTime();
    private ElapsedTime timezinceztart = new ElapsedTime();

    private String state = "initialization";
    private int launcherSpeed = 850;

    //I need to separate the code into functions or organize this better. State is nice, should be called step tho
    //cuz its not like state its different. states are repeated, steps are not. state machine != step machine

    private Pose currentPose;

    @Override
    public void loop() {
        if (follower.isBusy()){
            follower.update();
            follower_was_just_busy = true;
            currentPose = follower.getPose();
            telemetry.addData("current pose information","");
            telemetry.addData("heading",currentPose.getHeading());
            telemetry.addData("x",currentPose.getX());
            telemetry.addData("y",currentPose.getY());
        } else {
            follower_was_just_busy = false;
        }

        if (state == "initialization"){
            launchMotor.setVelocity(launcherSpeed);//this is done every frame.. not neccecary but i
            path = follower.pathBuilder()
                    .addPath(new BezierLine(firstPose, secondPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                    .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                    .build();
            follower.followPath(path);
            state = "first_movement";
        } else if (state == "first_movement"){
            if (!follower.isBusy()){
                state = "first_fire_loop";
            }
        } else if (state == "first_fire_loop"){
            if (fired_count<=3) {
                spin_launcher = true;
                if (launchMotor.getVelocity() == (launcherSpeed)){
                    if (timeSinceShot.seconds() > 1.5){
                        kick = true;
                        timeSinceShot.reset();
                        fired_count += 1;
                    }
                }
            } else {
                spin_launcher = false;
                state = "second_movement";
                path = follower.pathBuilder()
                        .addPath(new BezierLine(secondPose, thirdPose))
                        .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                        .build();
                follower.followPath(path);
            }
        } else if (state == "second_movement"){
            if (!follower.isBusy()){
                //do the next thing
            }
        }

        if (timeSinceShot.seconds() > 0.5) {
            kick = false;
        }

        if (spin_launcher){
            launchMotor.setVelocity(launcherSpeed); //ticks/s
        } else {
            launchMotor.setPower(0.05);
        }
        telemetry.addData("launchmotor targetv", launcherSpeed);
        telemetry.addData("launchmotor velocity",launchMotor.getVelocity());//ticks/s

        if (kick) {
            launchKickServo1.setPosition(LaunchServoAngle);
            launchKickServo2.setPosition(1-LaunchServoAngle);
        } else {
            launchKickServo1.setPosition(0);
            launchKickServo2.setPosition(1);
        }


//        telemetry.addData("p",currentCoefficients.p);
//        telemetry.addData("i",currentCoefficients.i);
//        telemetry.addData("d",currentCoefficients.d);
//        telemetry.addData("f",currentCoefficients.f);
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    }