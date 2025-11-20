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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

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

    public DcMotorEx intakeMotor;

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

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

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
        follower_code();
    }

    //exists purely for organisation, part of loop.
    private boolean follower_was_just_busy = true; //true if follower is not busy and it just was
    public void follower_code(){
        if (follower.isBusy()){
            follower_was_just_busy = true;
            follower.update();
        } else {
            pose_tracker.update();
        }
        Pose current_pose = follower.getPose();
        double x = current_pose.getX();//inches I think
        double y = current_pose.getY();
        double yaw = current_pose.getHeading(); //radians
        telemetry.addData("follower current pose","("+x+","+y+","+yaw+")");
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

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
    public int get_launcher_speed(){
        return launcherSpeed;
    }

    public void set_launcher_speed(int new_speed) {
        launcherSpeed = new_speed;
        launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
    }
    //ticks per second
    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(250,2,0,0); //was 200 p before flywheel

    //returns true each time it fires the artifact. indicates when the robot has decided to fire, not when the shot is clear.
    //do not move the instant this function returns true. You may attempt to fire again.
    //override shot will kick the artifact and reset the timer reguardless of whether it thinks it is ready




    public boolean launcher_code(boolean fire,boolean override_shot){
        //the return value of the function: did the robot fire the artifact
        boolean fired_this_tick = false;
        if (fire) {
            spin_launcher = true;
            //when the motor's velocity is equal - so when it fires, ball                will likley be slightly overshot.


            //TO BE IMPLEMENTED:
            
            //estimation for the launcher speed thingy

            //range = <how many ticks of speed you want to average>

            //average_speed = average_speed*1/(1-range) + current_speed(1/range)

            boolean right_speed_met = (launcherSpeed - rightLaunchMotor.getVelocity()) < 20.0;
            boolean left_speed_met = (launcherSpeed + leftLaunchMotor.getVelocity()) < 20.0;

//            telemetry.addData("right_speed_met",(rightLaunchMotor.getVelocity() - launcherSpeed));
//            telemetry.addData("left_speed_met",(-1*leftLaunchMotor.getVelocity() - launcherSpeed));
//            telemetry.addData("override shot",override_shot);

            if ((right_speed_met && left_speed_met)|| override_shot){  // //the right bumper serves as an override
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
    public void intake_code(){
    }

    public void set_intake_speed(int speed){
        intakeMotor.setPower(speed);
    }

    private Pose teleop_remembered_pose = new Pose(0,0,Math.toRadians(0));
    //run every tick with no arguments for ability to save and return to position in teleop
    public void teleop_return_to_position(){
        if (gamepad1.aWasPressed()){
            pose_tracker.update();
            teleop_remembered_pose = follower.getPose();
        }
        double x = teleop_remembered_pose.getX();//inches I think
        double y = teleop_remembered_pose.getY();
        double yaw = teleop_remembered_pose.getHeading(); //radians

        telemetry.addData("saved pose x,y,yaw","("+x+","+y+","+yaw+")");

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
    public void limelight_set_pose(){
        LLresult = limelight.getLatestResult();
        if (LLresult != null && LLresult.isValid()) {
            Pose3D limelight_botpose = LLresult.getBotpose();
            if (limelight_botpose != null) {
                double x = limelight_botpose.getPosition().x;
                double y = limelight_botpose.getPosition().y;
                YawPitchRollAngles limelight_orientation = limelight_botpose.getOrientation();
                double yaw = limelight_orientation.getYaw(AngleUnit.RADIANS);
                double meters_to_inches = 39.3701;
//                telemetry.addData("MT1 Location", "(" + x*meters_to_inches + ", " + y*meters_to_inches + ")");
//                telemetry.addData("MT1 Yaw", yaw);

                //this will likley be very off becasue the limelight is backwards..
                //the negitives and math.pi are to reverse the pose
                Pose limelight_pose = new Pose(-x*meters_to_inches,y*meters_to_inches,yaw);
                follower.setPose(limelight_pose);

                x = limelight_pose.getX();//inches I think
                y = limelight_pose.getY();
                yaw = limelight_pose.getHeading(); //radians
                telemetry.addData("limelight pose x,y,yaw","("+x+","+y+","+yaw+")");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    //manual control for drive, will use user input if pedro is not executing a task.
    public void drive_with_teleop(double forward,double strafe,double turn,double slowdown,boolean fire){
        if (follower_was_just_busy){
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            follower_was_just_busy = false;
        }

        //if we are trying to fire, line up with the goal.
        if (fire) {
            turn+= 0.03*LLresult.getTx(); //This could be a PID and it would be better
        }

        double slowdown_multiplier = 1 - (slowdown * .75);

        forward = forward * slowdown_multiplier;
        strafe = strafe * slowdown_multiplier;
        turn = turn * slowdown_multiplier;

        leftFront.setPower(forward - strafe - turn);
        leftBack.setPower(forward + strafe - turn);
        rightFront.setPower(forward + strafe + turn);
        rightBack.setPower(forward - strafe + turn);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}