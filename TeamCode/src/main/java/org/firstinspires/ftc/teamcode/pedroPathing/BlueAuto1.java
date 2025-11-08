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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BlueAuto1",preselectTeleOp = "DecodeTeleopMain")



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
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // this is a way to define a pose
    private final Pose endPose = new Pose(44, 0, 0); // this should be aiming at the back plate
//    private final Pose midPose = new Pose(15, 20, Math.toRadians(0));
//    private final Pose nextendPose = new Pose(20, 0, Math.toRadians(3.14));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        launchKickServo1 = hardwareMap.get(Servo.class,"lks1");
        launchKickServo2 = hardwareMap.get(Servo.class,"lks2");
        launchMotor = hardwareMap.get(DcMotorEx.class,"LaunchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.get(DcMotor.class,"intake");
    }

    public void start() {
        //spin up the flywheel
        intakeMotor.setPower(1);
        follower.activateDrive();

        //becasue we start with the intake facing backwards and the intake faces forwards, we start backwards. thus this is vital.
//        follower.setPose(new Pose(0,0,Math.toRadians(180)));

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
//                .addPath(new BezierLine(endPose, nextendPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        follower.followPath(path);
    }

    private double LaunchServoAngle = 0.35;
    private boolean kick = false;

    private int fired_count = 0;

    private Pose zkirtpoze = new Pose ();
    private boolean move1done = false;
    private boolean follower_was_just_busy = false;
    private ElapsedTime timesinceshot = new ElapsedTime();
    @Override
    public void loop() {
        launchMotor.setVelocity(900);
        if (follower.isBusy()){
            follower.update();
            follower_was_just_busy = true;
        } else {
            if (follower_was_just_busy){
                if ((launchMotor.getVelocity() >= 950)&&(timesinceshot.seconds()>2.0)){
                    kick = true;
                    timesinceshot.reset();
                }
                if (timesinceshot.seconds() > 0.5){
                    kick = false;
                    fired_count += 1;
                }
                if (fired_count >= 3){
                    //do nothing lol
                }
            }
        }

        if (kick) {
            launchKickServo1.setPosition(LaunchServoAngle);
            launchKickServo2.setPosition(1-LaunchServoAngle);
        } else {
            launchKickServo1.setPosition(0);
            launchKickServo2.setPosition(1);
        }
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

    }