package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;


//@TeleOp(name="DecodeTeleopMain")

public class CrossbowMain extends OpMode {

    public boolean launcher_freeze_movement = false;
    // Declare OpMode members.
    public TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Servo launchKickServo1;
    private Servo launchKickServo2;

    private GoBildaPinpointDriver pinpoint;

    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotorEx rightLaunchMotor;
    private DcMotorEx leftLaunchMotor;

    public PIDFCoefficients launcherCoefficients = new PIDFCoefficients(50,1,1,12);

    public DcMotorEx intakeMotor;

    Limelight3A limelight;

    //universal pedro stuff

    public static Follower follower;
    public static PoseTracker pose_tracker;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public int backboard_pipeline = 0;

    public int backboard_id = 20;

    public String team = "blue";

    Pose backboard_pose;

    public double apm;

    public void set_team(String team){
        if (team == "red"){
            backboard_pipeline = 1;
            backboard_id = 24;
            limelight.pipelineSwitch(backboard_pipeline);
            backboard_pose = new Pose(121.35, -7.37,0);
            apm = -1.0;
        } else if (team == "blue"){
            backboard_pipeline = 0;
            backboard_id = 20;
            limelight.pipelineSwitch(backboard_pipeline);
            backboard_pose = new Pose(121.35, 7.37,0);
            apm = 1.0;
        }
    }

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

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
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    //the initial remembered pose


    boolean spin_launcher = true;

    public boolean kick = false;
    public ElapsedTime timeSinceShot = new ElapsedTime();

    private int maxLauncherSpeed = 2200;
    private int minLauncherSpeed = 600;

    //for telemetry - I should really start breaking this stuff into functions so I can init variables near where they are used.
    double left_speed_at_kick = 0.0;
    double right_speed_at_kick = 0.0;


    public static double KickerLaunchAngle = 0.5;
    public static double KickerIdleAngle = 0;

    private int launcherSpeed = 780;
    public int get_launcher_speed(){
        return launcherSpeed;
    }

    public void set_launcher_speed(int new_speed) {
        launcherSpeed = new_speed;
        launcherSpeed = Math.max(Math.min(launcherSpeed,maxLauncherSpeed),minLauncherSpeed);
    }
    //ticks per second

    //pre 12/4/2025
//    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(290,3,0,0); //was 200 p before flywheel

   // 12/4/2025 -- I really need to be able to graph the zpeed.. tiz unfortunate.

    //returns true each time it fires the artifact. indicates when the robot has decided to fire, not when the shot is clear.
    //do not move the instant this function returns true. You may attempt to fire again.
    //override shot will kick the artifact and reset the timer reguardless of whether it thinks it is ready

    int left_speed_met_count = 1;
    int right_speed_met_count = 1;

    int desired_met_count = 5;

    public boolean trying_to_fire = false;


    private int launcher_moving_average_range = 15;
    private MovingAverage left_speed_average = new MovingAverage(launcher_moving_average_range); //this class was written by AI
    private MovingAverage right_speed_average = new MovingAverage(launcher_moving_average_range); //this class was written by AI

    public static int max_average_error = 20;
    public static int max_current_error = 20;

    //how fast can the robot be rotating and still fire?
    double max_angular_velocity = 0.1;

    double chasis_aim_turn = 0;

    public void set_motor_power_zero() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    double zero_power_turn = 0.001;

    double max_limelight_tx_error = 3;
    public boolean launcher_code(boolean fire,boolean override_shot){
        rangefind();
        //the return value of the function: did the robot fire the artifact
        boolean fired_this_tick = false;
        telemetry.addData("Launcher Target Velocity:", "\n"+launcherSpeed); // \n makes the text go down a line

        double right_current_speed = rightLaunchMotor.getVelocity();
        double left_current_speed = leftLaunchMotor.getVelocity();

        left_speed_average.addValue(left_current_speed);
        double left_speed_average_error = left_speed_average.getAverageError();
        panelsTelemetry.addData("l_speed_avg_error", left_speed_average_error);
        telemetry.addData("l_speed_avg_error",left_speed_average_error);

        right_speed_average.addValue(left_current_speed);
        double right_speed_average_error = left_speed_average.getAverageError();
        panelsTelemetry.addData("r_speed_avg_error", right_speed_average_error);
        telemetry.addData("r_speed_avg_error",right_speed_average_error);

        telemetry.addData("right_speed",right_current_speed);
        telemetry.addData("left_speed",left_current_speed);

//        telemetry.addData("left_speed_met_count",left_speed_met_count);
//        telemetry.addData("right_speed_met_count",right_speed_met_count);

        //This allows us to see the speeds of the left and right motor and tune the PIDs
        panelsTelemetry.addData("right_current_speed", right_current_speed);
        panelsTelemetry.addData("left_current_speed", left_current_speed);
        if (kick) {
            panelsTelemetry.addData("kick", 1.0*launcherSpeed);
        } else {
            panelsTelemetry.addData("kick", 0.0);
        }
        panelsTelemetry.addData("right_target_speed", launcherSpeed);
        panelsTelemetry.addData("left_target_speed", -launcherSpeed);

        //run all the checks even if we are not trying to fire!

        boolean right_speed_met = Math.abs(launcherSpeed - right_current_speed) < max_current_error;
        right_speed_met = right_speed_met && (Math.abs(right_speed_average_error)<max_average_error);
//
        boolean left_speed_met = Math.abs(launcherSpeed + left_current_speed) < max_current_error;
        left_speed_met = left_speed_met && (Math.abs(left_speed_average_error)<max_average_error);

        double chasis_angular_velocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        boolean speed_ready = right_speed_met && left_speed_met;
        boolean limlight_ready = (Math.abs(tx) < max_limelight_tx_error)&&LLresult.isValid();
        boolean angular_velocity_acceptable = Math.abs(chasis_angular_velocity) < max_angular_velocity;

        telemetry.addData("speed_ready",speed_ready);
        if (limlight_ready) {
            telemetry.addData("limelight_ready,tx",tx);
        } else if (!LLresult.isValid()){
            telemetry.addData("limelight_error_tag","No Tag");
        } else {
            telemetry.addData("limelight_error,tx",tx);
        }

        panelsTelemetry.addData("chasis_angular_velocity",chasis_angular_velocity);
        telemetry.addData("chasis_angular_velocity",chasis_angular_velocity);

        if (fire) {
            trying_to_fire = true;
            spin_launcher = true;

            //do the lineup

            chasis_aim_turn= 0.03*(tx); //This could be a PID and it would be better
            double cats = chasis_aim_turn/Math.abs(chasis_aim_turn); //chasis aim turn sign

            leftFront.setPower(leftFront.getPower()+chasis_aim_turn);//+(zero_power_turn*cats));
            leftBack.setPower(leftBack.getPower()+chasis_aim_turn);//+(zero_power_turn*cats));
            rightFront.setPower(rightFront.getPower()-chasis_aim_turn);//-(zero_power_turn*cats));
            rightBack.setPower(rightBack.getPower()-chasis_aim_turn);//-(zero_power_turn*cats));

            //take the shot
            if ((speed_ready && limlight_ready && angular_velocity_acceptable) || override_shot){  // //the right bumper serves as an override
                launcher_freeze_movement = true;
                if (timeSinceShot.seconds() > 1.15){
                    kick = true;
                    timeSinceShot.reset();
                    //debug information - motor 2 is left, motor 1 is right
                    //
                    left_speed_at_kick = left_current_speed;
                    right_speed_at_kick = right_current_speed;
                    fired_this_tick = true;
                }
            }
        } else {
            spin_launcher = false;
            trying_to_fire = false;
            telemetry.addData("speed_ready"," -N/A-");
            telemetry.addData("limelight_ready"," -N/A-");
            telemetry.addData("bias",limelight_x_offset);
        }
        telemetry.addData("left_speed_at_kick",left_speed_at_kick);
        telemetry.addData("right_speed_at_kick",right_speed_at_kick);


        double kicker_extension_time = 0.3;
        if (timeSinceShot.seconds() > kicker_extension_time) {
            kick = false;
            launcher_freeze_movement = false;
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

//        telemetry.addData("launchmotor1 velocity", rightLaunchMotor.getVelocity());//ticks/s
//        telemetry.addData("launchmotor2 velocity", leftLaunchMotor.getVelocity());//ticks/s

        return fired_this_tick;
    }

    public void rangefind(){
        if (estimated_distance < 100){
            launcherSpeed = 780;
            limelight_x_offset = 0;
        } else {
            launcherSpeed = 960;
            limelight_x_offset = -2*apm;
        }
    }

    public LLResult LLresult;

    public double tx = 0.0;
    //how much to offset the shot
    public double limelight_x_offset = 0.0;

    public double estimated_distance = 0;
    public void limelight_code(){
        //limelight stuff - should always run

        //This will probably need an offset
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));
        LLresult = limelight.getLatestResult();
        telemetry.addData("current pipeline",LLresult.getPipelineIndex());
        if ((LLresult != null) && LLresult.isValid()) {
            tx = LLresult.getTx()+limelight_x_offset; // How far left or right the target is (degrees)
            telemetry.addData("tx",tx);

//            telemetry.addData("tx",tx);


            //https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
            //how I made this.. I should just use megatag 2..

            //after some testing this looks pretty stable! gonna keep it for this comp

            double ty = LLresult.getTy();
            double ta = LLresult.getTa(); // How big the target looks (0%-100% of the image)
            double limelightMountAngleDegrees = 19.0;
            double targetOffsetAngle_Vertical = ty;
            double limelight_height = 11.5;
            double goal_tag_height = 29.5;

            double angleToGoalDegrees = targetOffsetAngle_Vertical+limelightMountAngleDegrees;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            estimated_distance = (goal_tag_height-limelight_height) / Math.tan(angleToGoalRadians);
            telemetry.addData("estimated distance w/ angles",estimated_distance);
            // gives the x offset from the limelight
//            telemetry.addData("Target X", tx);
        } else {
            telemetry.addData("Limelight", "No Targets");
        //do a \n for each line of telemetry you put above so wheather or not lime has a target it takes the same space.
            tx = 0;
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
        if (LLresult != null && LLresult.isValid()) {
            Pose3D limelight_botpose = LLresult.getBotpose_MT2();
            if (limelight_botpose != null) {
                double meters_to_inches = 39.3701;
                double x = limelight_botpose.getPosition().x*meters_to_inches;
                double y = limelight_botpose.getPosition().y*meters_to_inches;
                YawPitchRollAngles limelight_orientation = limelight_botpose.getOrientation();
                double yaw = limelight_orientation.getYaw(AngleUnit.RADIANS);

//                telemetry.addData("MT1 Location", "(" + x*meters_to_inches + ", " + y*meters_to_inches + ")");
//                telemetry.addData("MT1 Yaw", yaw);

                //this will likley be very off becasue the limelight is backwards..
                //the negitives and math.pi are to reverse the pose
                Pose pedro_limelight_pose = new Pose(x,y,yaw);
                follower.setPose(pedro_limelight_pose);

                x = pedro_limelight_pose.getX();//inches I think
                y = pedro_limelight_pose.getY();
                yaw = pedro_limelight_pose.getHeading(); //radians

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

        double slowdown_multiplier = 1 - (slowdown * .75);


        //had to make it negitive for now to account for weird pedro reversal stuff. figure this out more later.
        forward = -forward * slowdown_multiplier;
        strafe = -strafe * slowdown_multiplier;
        turn = -turn * slowdown_multiplier;

        leftFront.setPower(forward - strafe - turn);
        leftBack.setPower(forward + strafe - turn);
        rightFront.setPower(forward + strafe + turn);
        rightBack.setPower(forward - strafe + turn);

        if (launcher_freeze_movement){
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public int int_clamp(int value,int min,int max){
        return Math.min((Math.max(value,min)),max);
    }
}