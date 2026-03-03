package org.firstinspires.ftc.teamcode.pedroPathing;

//import com.bylazar.field.PanelsField;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;

//@TeleOp(name="DecodeTeleopMain")

    ///IMPORTANT DEBUG INFO - put these into your browser while on robot wifi to see them

    //here is the IP address for PANELS and for LIMELIGHT

    //  http://192.168.43.1:5801/    - limelight

    //  http://192.168.43.1:8001/    - panels


@Configurable
public class CrossbowMain extends OpMode {

//    public PanelsField panelsField = PanelsField.INSTANCE;

    public static double the_time_it_takes_to_open_the_door_in_seconds = 0;



    //these rough estimates are now outdated and should only be used as a starting point

//    public static int super_near_shot_speed = 1120;
    public static int near_shot_speed = 1200;

//    public static int far_shot_speed = 1550;



    public boolean launcher_freeze_movement = false;
    // Declare OpMode members.
    public TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Servo launchKickServo1;
    private Servo launchKickServo2;

    protected GoBildaPinpointDriver pinpoint;

///turret variables
    private DcMotorEx turret_motor;
    private DcMotor.RunMode turret_motor_runmode = DcMotor.RunMode.RUN_USING_ENCODER;

    private double turret_motor_ppr = 537.7;

    private int turret_large_gear_teeth = 72;

    private int turret_small_gear_teeth = 12;

    private double turret_ppr;

    private int turret_max_ticks;

    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotorEx rightLaunchMotor;
    private DcMotorEx leftLaunchMotor;

    private TouchSensor magnetic_limit_switch_left;
    private TouchSensor magnetic_limit_switch_right;

    public PIDFCoefficients launcherCoefficients = new PIDFCoefficients(100,0,0,12.9);

    public DcMotorEx intakeMotor;

    Limelight3A limelight;

    //universal pedro stuff

    protected Pose pedro_pose_from_limelight;
    protected Pose current_pedro_pose;
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
            backboard_pipeline = 3;
            backboard_id = 24;
            limelight.pipelineSwitch(backboard_pipeline);
            backboard_pose = new Pose(121.35, -7.37,0);
            apm = -1.0;
        } else if (team == "blue"){
            backboard_pipeline = 2;
            backboard_id = 20;
            limelight.pipelineSwitch(backboard_pipeline);
            backboard_pose = new Pose(121.35, 7.37,0);
            apm = 1.0;
        }
    }

    @Override
    public void init() {

    /// pedro
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        follower = Constants.createFollower(hardwareMap);
        pose_tracker = follower.getPoseTracker();

    /// limelight camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();


    /// turret rotation
        magnetic_limit_switch_left = hardwareMap.get(TouchSensor.class,"magL");
        magnetic_limit_switch_right = hardwareMap.get(TouchSensor.class,"magR");

        turret_motor = hardwareMap.get(DcMotorEx.class, "turret");
        turret_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_motor.setMode(turret_motor_runmode);
        turret_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_ppr = turret_motor_ppr*turret_large_gear_teeth/turret_small_gear_teeth;
        turret_max_ticks = (int) Math.floor(turret_ppr*(3/4));

    /// launch motors
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm1");
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //don't waste energy braking on 0 power bro

        leftLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm2");
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    /// door servos
        launchKickServo1 = hardwareMap.get(Servo.class,"lks1");
        launchKickServo2 = hardwareMap.get(Servo.class,"lks2");
        //servo start position
        launchKickServo1.setPosition(0);
        launchKickServo2.setPosition(1);
    /// drive motors

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    /// intake
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

    /// end telemetry
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
        limelight.start();
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
        intake_code();
        if (update_chasis_pid_toggle){
            update_chasis_pid_toggle = false;
            chasis_pid = new PID(aiming_pid_coeficients[0], aiming_pid_coeficients[1], aiming_pid_coeficients[2]);
        }

//        panelsTelemetry.addData("magnetic_limit_switch_left", magnetic_limit_switch_left.getValue());
//        panelsTelemetry.addData("magnetic_limit_switch_right",magnetic_limit_switch_right.getValue());
    }

    //exists purely for organisation, part of loop.
    private boolean follower_was_just_busy = true; //true if follower is not busy and it just was
    public void follower_code(){
        current_pedro_pose = follower.getPose();

        panelsTelemetry.addData("current_pedro_pose x",current_pedro_pose.getX());
        panelsTelemetry.addData("current_pedro_pose y",current_pedro_pose.getY());
        panelsTelemetry.addData("current_pedro_pose heading",current_pedro_pose.getHeading());

        if (follower.isBusy()){
            follower_was_just_busy = true;
            follower.update();
        } else {
            pose_tracker.update();
        }


        //draw the pose.. don't know how.
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    //the initial remembered pose


    protected boolean spin_launcher = false;

    public boolean kick = false;
    public ElapsedTime timeSinceShot = new ElapsedTime();

    private int maxLauncherSpeed = 2200;
    private int minLauncherSpeed = 600;

    //for telemetry - I should really start breaking this stuff into functions so I can init variables near where they are used.
    double left_speed_at_kick = 0.0;
    double right_speed_at_kick = 0.0;


    public static double KickerLaunchAngle = 0.3;
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


    private int launcher_moving_average_range = 8;
    private int limelight_error_moving_average_range = 8;
    private MovingAverage left_speed_average = new MovingAverage(launcher_moving_average_range); //this class was written by AI
    private MovingAverage right_speed_average = new MovingAverage(launcher_moving_average_range); //this class was written by

    private MovingAverage limelight_error_average = new MovingAverage(limelight_error_moving_average_range); //this class was written by AI

    public static int max_average_error = 15;
    public static int max_current_error = 40; //there is no 30 so this is goofy but whatever
    public static int max_current_error_lazy = 300; //there is no 30 so this is goofy but whatever

    //how fast can the robot be rotating and still fire?
    public static double max_angular_velocity = 12;
    public static double max_linear_velocity = 12;

    public void set_motor_power_zero() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    double zero_power_turn = 0.001;

    public static double max_limelight_tx_error_init = 0.5;
    public static double max_limelight_tx_error_sustain = 2;

    public static double max_limelight_average_error = 1;
    public static boolean debug_kicker = false;
    public static boolean override_kick = false;

    protected boolean open_door = false;

    protected ElapsedTime door_open_timer = new ElapsedTime();


    double limelight_chasis_rotation_multiplier = 0.02; //old system
    private PID chasis_pid = new PID(0.01,0,0);

    public static double zero_power_movement_constant = 0.08;
    public static double[] aiming_pid_coeficients = {
            0.02,0.001,0
    };
    public void update_chasis_pid(double P, double I, double D){
        chasis_pid = new PID(P,I,D);
    }

    public static boolean update_chasis_pid_toggle = false;
    protected double goal_aim_pid_output = 0;
        public void launcher_code(boolean fire,boolean override_shot){
        rangefind();
        //the return value of the function: did the robot fire the artifact
        telemetry.addData("Launcher Target Velocity:", "\n"+launcherSpeed); // \n makes the text go down a line

        double right_current_speed = rightLaunchMotor.getVelocity();
        double left_current_speed = leftLaunchMotor.getVelocity();

        left_speed_average.addValue(left_current_speed);
        double left_speed_average_error = left_speed_average.getAverageError();
        panelsTelemetry.addData("l_speed_avg_error", left_speed_average_error);
//        telemetry.addData("l_speed_avg_error",left_speed_average_error);

        right_speed_average.addValue(left_current_speed);
        double right_speed_average_error = left_speed_average.getAverageError();
        panelsTelemetry.addData("r_speed_avg_error", right_speed_average_error);
//        telemetry.addData("r_speed_avg_error",right_speed_average_error);

        //This allows us to see the speeds of the left and right motor and tune the PIDs
        panelsTelemetry.addData("right_current_speed", right_current_speed);
        panelsTelemetry.addData("left_current_speed", left_current_speed);
        panelsTelemetry.addData("kick",bool_spike(kick));

        panelsTelemetry.addData("right_target_speed", launcherSpeed);
        panelsTelemetry.addData("left_target_speed", -launcherSpeed);

        //run all the checks even if we are not trying to fire!

        //new system does not need the superchecks

        boolean right_speed_met = Math.abs(launcherSpeed - right_current_speed) < max_current_error;
        right_speed_met = right_speed_met && (Math.abs(right_speed_average_error)<max_average_error);
//
        boolean left_speed_met = Math.abs(launcherSpeed + left_current_speed) < max_current_error;
        left_speed_met = left_speed_met && (Math.abs(left_speed_average_error)<max_average_error);

        double chasis_angular_velocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        double chasis_linear_velocity_odd = pinpoint.getVelY(DistanceUnit.INCH)+pinpoint.getVelY(DistanceUnit.INCH);
        panelsTelemetry.addData("chasis_linear_velocity_odd",chasis_linear_velocity_odd);

        boolean flywheel_speed_acceptable = right_speed_met || left_speed_met;



        boolean limelight_average_error_acceptable = Math.abs(limelight_error_average.getAverageError()) < max_limelight_average_error;

        boolean limelight_error_acceptable_init = limelight_average_error_acceptable && (Math.abs(tx) < max_limelight_tx_error_init)&&LLresult.isValid();
        boolean limelight_error_acceptable_sustain = (Math.abs(tx) < max_limelight_tx_error_sustain)&&LLresult.isValid();

        boolean angular_velocity_acceptable = Math.abs(chasis_angular_velocity) < max_angular_velocity;
        boolean linear_velocity_acceptable = Math.abs(chasis_linear_velocity_odd) < max_linear_velocity;



        telemetry.addData("speed_ready",flywheel_speed_acceptable);
        if (limelight_error_acceptable_init) {
            telemetry.addData("limelight_ready,tx",tx);
        } else if (!LLresult.isValid()){
            telemetry.addData("limelight_error_tag","No Tag");
        } else {
            telemetry.addData("limelight_error,tx",tx);
        }

        boolean left_speed_met_easy = Math.abs(launcherSpeed + left_current_speed) < max_current_error_lazy;
        boolean right_speed_met_easy = Math.abs(launcherSpeed - right_current_speed) < max_current_error_lazy;
        boolean basic_speed_acceptable = left_speed_met_easy&&right_speed_met_easy;

        telemetry.addData("chasis_angular_velocity",chasis_angular_velocity);

        panelsTelemetry.addData("chasis_angular_velocity",chasis_angular_velocity);
        panelsTelemetry.addData("chasis_linear_velocity_odd",chasis_linear_velocity_odd);


        panelsTelemetry.addData("basic_speed_acceptable",bool_spike(basic_speed_acceptable));
        panelsTelemetry.addData("flywheel_speed_acceptable",bool_spike(flywheel_speed_acceptable));
        panelsTelemetry.addData("limelight_error_acceptable",bool_spike(limelight_error_acceptable_init));
        panelsTelemetry.addData("angular_velocity_acceptable",bool_spike(angular_velocity_acceptable));
        panelsTelemetry.addData("linear_velocity_acceptable",bool_spike(linear_velocity_acceptable));
        panelsTelemetry.addData("override_shot",bool_spike(override_shot));



        goal_aim_pid_output = chasis_pid.update(0,tx);

        //goal_aim_pid_output = goal_aim_pid_output; //integration of Zero power

        //goal_aim_pid_output = 0; //integration of Zero power movement

//        double zeropower_deadzone = 0.01;
//
//        if (chasis_pid_output > zeropower_deadzone){
//            goal_aim_pid_output +=zero_power_movement_constant;
//        } else if (chasis_pid_output < -zeropower_deadzone){
//            goal_aim_pid_output -=zero_power_movement_constant;
//        }

        if (fire) {
            trying_to_fire = true;
            spin_launcher = true;

            //do the lineup

            //take the shot - once you've started, don't stop!

            boolean non_flywheel_conditions = (linear_velocity_acceptable && angular_velocity_acceptable && !follower.isBusy());

            boolean open_door_conditions = ((limelight_error_acceptable_init && flywheel_speed_acceptable && non_flywheel_conditions) || (override_shot && basic_speed_acceptable));
            //If the speed goes back down.. too bad. door stays open. not in use rn because I think it causes missing when we get defended.
            boolean keep_door_open_conditions = (non_flywheel_conditions && limelight_error_acceptable_sustain && basic_speed_acceptable);

            open_door = open_door_conditions || (keep_door_open_conditions && open_door);

            if (open_door){  // //the right bumper serves as an override
                    launcher_freeze_movement = true;
            } else {
                door_open_timer.reset();
            }
        } else {
            launcher_freeze_movement = false;
            if (gamepad1.b){ spin_launcher = false;}
            trying_to_fire = false;
            open_door = false;
            telemetry.addData("speed_ready"," -N/A-");
            telemetry.addData("limelight_ready"," -N/A-");
            telemetry.addData("bias",limelight_x_offset);
        }
        telemetry.addData("left_speed_at_kick",left_speed_at_kick);
        telemetry.addData("right_speed_at_kick",right_speed_at_kick);

        //debug
        if (debug_kicker) {
            kick = override_kick;
            if (gamepad1.xWasPressed()){
                override_kick = !override_kick;
            }
        }

        if (open_door) {
            launchKickServo1.setPosition(KickerLaunchAngle);
            launchKickServo2.setPosition(1- KickerLaunchAngle);
        } else {
            if(!fire) {
                launchKickServo1.setPosition(KickerIdleAngle);
                launchKickServo2.setPosition(1 - KickerIdleAngle);
            }
        }

        if (spin_launcher){
            rightLaunchMotor.setPower(1); //try to reduce how often we set these later, I hear it can be taxing.
            rightLaunchMotor.setVelocity(launcherSpeed); //ticks/s
            leftLaunchMotor.setPower(1);
            leftLaunchMotor.setVelocity(-1*launcherSpeed); //ticks/s
        } else {
            rightLaunchMotor.setPower(0);
            leftLaunchMotor.setPower(0);
        }

//        telemetry.addData("launchmotor1 velocity", rightLaunchMotor.getVelocity());//ticks/s
//        telemetry.addData("launchmotor2 velocity", leftLaunchMotor.getVelocity());//ticks/s
    }


    //rangefinder curve fit constants
//https://www.desmos.com/calculator/wz3ai30ujx
    public static double[] rangefinder_constants = {
            0.000588967,
            3.4,
            1000
    };
    public void rangefind(){
        double unrounded_launcher_speed = rangefinder_constants[0]*Math.pow(estimated_distance,2)+rangefinder_constants[1]*estimated_distance+rangefinder_constants[2];
        launcherSpeed = Math.round((long) (unrounded_launcher_speed/20))*20;


        panelsTelemetry.addData("launcherTargetSpeed",launcherSpeed);

//        if (estimated_distance < 50){
//            launcherSpeed = super_near_shot_speed;
//            limelight_x_offset = 0;
//        } else if (estimated_distance < 130){
//            launcherSpeed = near_shot_speed;
//            limelight_x_offset = 0;
//        } else {
//            launcherSpeed = far_shot_speed;
//            limelight_x_offset = -2*apm;
//        }
    }

    public LLResult LLresult;

    public double launch_angle_error = 0.0;
    private double tx = 0.0;
    //how much to offset the shot
    public double limelight_x_offset = 0.0;

    public double estimated_distance = 0;

    public static double min_turret_power_limit = 0.05;

    public void turret_spin_to_rotation_radians(double angle){
        panelsTelemetry.addData("turret_target_radians",angle);

        angle = (angle + Math.PI)%(Math.PI*2)-Math.PI;

        double temp_tick_target = (angle/(Math.PI*2))*turret_ppr; //1 should be turret ppr but its evil?

        int tick_target = (int) Math.round(temp_tick_target);

        turret_motor.setTargetPosition(tick_target);

        panelsTelemetry.addData("turret_target_tick",temp_tick_target);


        if (turret_motor_runmode != DcMotor.RunMode.RUN_TO_POSITION){
            turret_motor_runmode = DcMotor.RunMode.RUN_TO_POSITION;
            turret_motor.setMode(turret_motor_runmode);
        }

        double turret_position_ticks = turret_motor.getCurrentPosition();

        if (
                ((magnetic_limit_switch_left.getValue() == 1)&&(turret_position_ticks<tick_target))
                ||
                ((magnetic_limit_switch_right.getValue() == 1)&&(turret_position_ticks>tick_target))
        ){
            turret_motor.setPower(0);
        } else {
            turret_motor.setPower(1);
        }
//            //write offset code to handle this later

//        if (magnetic_limit_switch_left.getValue() == 1){
//            //write offset code to handle this
//        }
//        if (magnetic_limit_switch_right.getValue() == 1){
//            //write offset code to handle this
//        }

        //this could be smarter (more math efficient) by finding a ticks-per-radian instead of ticks-per-revolution

        panelsTelemetry.addData("turret_rotation_degrees",get_turret_rotation_degrees());
    }
    public void spin_turret_simple(double power){
        if (turret_motor_runmode != DcMotor.RunMode.RUN_USING_ENCODER){
            turret_motor_runmode = DcMotor.RunMode.RUN_USING_ENCODER;
            turret_motor.setMode(turret_motor_runmode);
        }
        if (magnetic_limit_switch_left.getValue() == 1){
            power = Math.max(power, -min_turret_power_limit);
        }
        if (magnetic_limit_switch_right.getValue() == 1){
            power = Math.min(power, min_turret_power_limit);
        }
        turret_motor.setPower(power);
        panelsTelemetry.addData("turret_rotation_degrees",get_turret_rotation_degrees());
    }

    public double get_turret_rotation_degrees(){
        return (turret_motor.getCurrentPosition()/3226.2)*360;
    }
    public void limelight_code(){
        //limelight stuff - should always run

        //This will probably need an offset
        limelight.updateRobotOrientation((pinpoint.getHeading(AngleUnit.DEGREES)+get_turret_rotation_degrees())%360);
        //check if the compas is clockwise or not and make the turret rotation match that!

        LLresult = limelight.getLatestResult();
        telemetry.addData("current pipeline",LLresult.getPipelineIndex());

        if ((LLresult != null) && LLresult.isValid()) {
            tx = LLresult.getTx()+limelight_x_offset; // How far left or right the target is (degrees)

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

            //turing the limelight pose into a pedro pose. was using MT2, now using MT1 for a bit. less acurate but not reliant on a gyro.
            Pose3D limelight_botpose = LLresult.getBotpose_MT2();
            Position llbpposition = limelight_botpose.getPosition();
            Pose2D llpose2d = new Pose2D(DistanceUnit.METER,llbpposition.x,llbpposition.y,AngleUnit.DEGREES,limelight_botpose.getOrientation().getYaw());
            pedro_pose_from_limelight = PoseConverter.pose2DToPose(llpose2d,PedroCoordinates.INSTANCE);

            panelsTelemetry.addData("pedro_pose_from_limelight x",pedro_pose_from_limelight.getX());
            panelsTelemetry.addData("pedro_pose_from_limelight y",pedro_pose_from_limelight.getY());
            panelsTelemetry.addData("pedro_pose_from_limelight heading",pedro_pose_from_limelight.getHeading());

            Position limelight_position = limelight_botpose.getPosition();
            Position goal_position = new Position(DistanceUnit.INCH,65,65,0,0);

            double dx = goal_position.x-limelight_position.x;
            double dy = goal_position.y-limelight_position.y;

            double desired_angle = Math.atan2(dx,dy);
            //panelsTelemetry.addData("desired_angle",desired_angle);
            //panelsTelemetry.addData("current_angle",pinpoint.getHeading(AngleUnit.RADIANS));
        } else {
            telemetry.addData("Limelight", "No Targets");
            pedro_pose_from_limelight = null;
        //do a \n for each line of telemetry you put above so wheather or not lime has a target it takes the same space.
            tx = 0;
        }
        limelight_error_average.addValue(tx);
        telemetry.addData("tx",tx);
    }

    protected boolean spin_intake = false;
    protected boolean reverse_intake = false;

    private int reverse_multiplier = 1;
    public void intake_code(){
        if (reverse_intake && !open_door){
            reverse_multiplier = -1;
        } else {
            reverse_multiplier = 1;
        }

        //it's a 312 so 537.7 PPR at the Output Shaft. 5.2 RPS (max) would be 2796.04 or about 2800.
        if (((spin_intake||reverse_intake)&&(!open_door))||(open_door && (door_open_timer.seconds() > the_time_it_takes_to_open_the_door_in_seconds))){
            set_intake_speed(2000*reverse_multiplier);
        } else {
            set_intake_speed(0);
        }
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

                x = pedro_limelight_pose.getX() - 72;//inches I think
                y = pedro_limelight_pose.getY() + 72; // hopefully this should provide the translated coords
                yaw = pedro_limelight_pose.getHeading(); //radians

                telemetry.addData("limelight pose x,y,yaw","("+x+","+y+","+yaw+")");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    //manual control for drive, will use user input if pedro is not executing a task.
    public void manual_drive(double forward, double strafe, double turn, double slowdown){
        if (follower_was_just_busy){
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            follower_was_just_busy = false;
        }

        //if we are trying to fire, line up with the goal.

        //slowdown should probably be in teleop..
        double slowdown_multiplier = 1 - (slowdown * .75);

        //had to make it negitive for now to account for weird pedro reversal stuff. figure this out more later.
        forward = -forward*slowdown_multiplier;
        strafe = -strafe*slowdown_multiplier;
        turn = (-turn*slowdown_multiplier);

        //chasis aim replaced with turret aim
//      turn += goal_aim_pid_output;

        //field centric
//        double pinpoint_heading = follower.getHeading();
//
//        panelsTelemetry.addData("pinpoint_heading",pinpoint_heading);
//
//        double field_forward = forward*Math.sin(pinpoint_heading)-strafe*Math.cos(pinpoint_heading);
//        double field_strafe = forward*Math.cos(pinpoint_heading)+strafe*Math.sin(pinpoint_heading);
//
//        forward = field_forward;
//        strafe = field_strafe;

        leftFront.setPower(forward - strafe - turn);
        leftBack.setPower(forward + strafe - turn);
        rightFront.setPower(forward + strafe + turn);
        rightBack.setPower(forward - strafe + turn);

//        if (launcher_freeze_movement){
//            leftFront.setPower(0);
//            leftBack.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//        }
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

    public int bool_spike(boolean value){
        if (value){
            return (100);
        } else {
            return (0);
        }
    }
}