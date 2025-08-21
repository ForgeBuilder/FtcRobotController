package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="TeleopMain")

public class TeleopMain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    // Initialise motor variables
    
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor rightSlide = null;
    private DcMotor rightArm = null;
    
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor leftSlide = null;
    private DcMotor leftArm = null;
    
    //how many encoder ticks are needed for 1 degree of turning?
    private double armEncoderMultiplyer = 2650.0/90.0; //encoder target required for 90 degree turn / 90.0
    //this used to be like 1725 before the switch
    
    private IMU leftIMU = null;
    
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    
    private String servo_state = "release";
    private boolean toggle = false;

    private double armTarget = 0;
    private int slideTarget = 0;
    private int armSpeed = 10;
    private int slideSpeed = 10;
    
    private boolean slideOverride = false;
    
    //States are lowercase and cammelcase. Right trigger is used to exit current task and return to manual.
    private String arm_state = "manual";
    //valid states are "manual"
    private String drive_state = "manual";
    //valid states are "manual","hang","basket_high","basket_low","specimine_high","specimine_low"
    
    private double armPower = 1;
    private double slidePower = 1;
    
    //state machine stuff
    private boolean armReady = false;
    private boolean slideReady = false;
    private double lastSlideInput = 0;
    private double lastArmInput = 0;
    
    private int step = -1;
    private boolean readyStep = false;
    private double hangTime1 = 0;
    //\\state machine stuff\\
    
    
    private IMU.Parameters IMUparameters;
    private double pitch = 0.0;
    private Map<String, Integer> toggles = new HashMap<>();
    
    private boolean rb_last_input;
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
     
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //IMU stuff
        leftIMU = hardwareMap.get(IMU.class,"imu");
        
        IMUparameters = new IMU.Parameters(
         new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.UP,
              RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        leftIMU.initialize(IMUparameters);
        
        //resets the IMU rotation values. I should change this so
        //its mapped to a button or happens durring auto
        leftIMU.resetYaw();
    
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
    
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        
        leftGripper = hardwareMap.get(Servo.class,"leftGripper");
        rightGripper = hardwareMap.get(Servo.class,"rightGripper");
    
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        //Make every motor break when at power 0
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Setup arm swing
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        rightArm.setTargetPosition(0);
        leftArm.setTargetPosition(0);
        
        rightArm.setPower(1);
        leftArm.setPower(1);
        
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //setup slide
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        // make the right side wheels reversed
        // leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        //State Machine
        
        if (true){ // if(true) is for organisational indent
            
            //ARM STATE
            
            //hang - requires all buttons! It's a little weird because it's an arm state but driver is the one who starts it.
            if (gamepad1.x){
                arm_state = "hang";
            }
            
            if (gamepad2.a){
                arm_state = "specimine_high";
            }
            
            //wrong name convention
            if (gamepad2.x){
                arm_state = "high_basket"; //I should use the dpad for choosing what to do. 
                //dpad values for high b, low b, high s and low s.
            }
            
            //make sure this is the last if statement
            if ((gamepad1.right_trigger > 0.1)){
                state_machine_clear_arm();
            }
            
            if ((gamepad2.right_trigger > 0.1)&&(arm_state!="hang")){
                state_machine_clear_arm();
            }
            //This is somwhat important, don't know were it goes vv
            //readyStep = false; //stops the thing from going to infinity steps
            //also, for your manual overide, REMEMBER TO PRESS "A"!
        }
        
        if (drive_state == "manual"){
            //manual controll for drive
            
            double forward = -gamepad1.left_stick_y;
            double Strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            
            //slide recalibrate..
            
            //power slides so they retract and stop/reset encoders with the top bumpers
            
            //driver 1's slowmode
            
            double slowdown = 1-(gamepad1.left_trigger*.75);
            
            
            forward = forward*slowdown;
            Strafe = Strafe*slowdown;
            turn = turn*slowdown;
            
            
            //setpower for drive
            leftFront.setPower(forward + Strafe + turn);
            leftBack.setPower(forward - (Strafe - turn));
            rightFront.setPower(forward - (Strafe + turn));
            rightBack.setPower(forward + (Strafe - turn));
        }
        
        
        
        //manual controll for arm
        //freindly note- we use 117 RPM rpm motors gobilda motor for the swinging arm mechanisum. keyword spam for frantic searching later.
        if (arm_state == "manual"){
            
            //slide fixer
            
            if (gamepad2.right_bumper && !rb_last_input){
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slidePower = -0.3;
            } else if (!gamepad2.right_bumper && rb_last_input){
                slideTarget = 0;
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidePower = 1;
            }
            rb_last_input = false;
            rb_last_input = gamepad2.right_bumper;
            
            //servo controll
            if (gamepad2.y){
                if (toggle){
                    if (servo_state == "release"){
                        servo_state = "grip";
                    }else{
                        servo_state = "release";
                    }
                    
                    toggle = false;
                }
            } else {
                toggle = true;
            }
            
            
            
            //gets gamepad input
            double slideInput = -gamepad2.left_stick_y;
            double armInput = gamepad2.right_stick_y;
            
        //slowmode for arm swing
            
            //slowmode is forced if you are extended too far to prevent gear slippage
            //note slipping can still occur, it's an underlying hardware issue.
            //its between the big and small gear. 12/5/2024
            if (gamepad2.left_bumper || slideTarget > 500){
                armSpeed = 5;
            } else {
                armSpeed = 30;
            }
        //slowmode for slide. no overide for this one (:
        
        
            if (gamepad2.left_bumper){
                armInput = (armInput/3);
            } else {
                armInput = (armInput);
            }
            
            //integrate gamepad input to target position, don't let it go
            //into places it should not with limits.
            
            //this is where armtarget is set durring tele
            
            
            //This system is BAD. I should make it so just whenever joystick is held we
            //switch to power based movement, and when I let go we go back to target. ehh that does
            //mess with the limits tho.. Possibly a combination of the two. target much further ahead, but use the
            //low power setting. 
            
            //didn't work. more exeriments required.
            
            //This is where the arm's target is set and it's speed governed.
            //called only durring manual 
            if (Math.abs(lastArmInput) > 0) {
                armTarget = rightArm.getCurrentPosition()/armEncoderMultiplyer+(armInput*300);
                armPower = 0.1+armInput*0.9;
            }
            lastArmInput = armInput;
            
            if (armTarget > 90.0) {
                armTarget = 90.0;
            }
            if (armTarget < 0 && !slideOverride) {
                armTarget = 0;
            }
            if (armTarget < -78.26) {
                armTarget = -78.26;
            }
            
            
            /*
            * not needed due to hardware rework vvv
            */
            // if (slideTarget < 400 && armTarget > 775) { //prevents arm from shoving down if there would be collisions
            //     armTarget = 775;
            //     slideTarget = 400;
            // }
            
            
            //slide overide - allows illigal movement! use carefully!
            if(gamepad2.right_trigger > 0.1) {
                slideOverride = true;
            }
            if(gamepad2.left_trigger > 0.1) {
                slideOverride = false;
            }
            
            //the reason this is here is so that when a state ends,
            //the user will only overide the last command if they
            //start pressing things.
            
            //if there is input, change the target
            if (Math.abs(lastSlideInput) > 0) {
                //get a number for input direction that is either -1, 0, or 1.
                int inputDirection = (int) Math.floor(slideInput/Math.abs(slideInput));
                slideTarget = rightSlide.getCurrentPosition()+300*inputDirection;
                slidePower = 0.1+(Math.abs(slideInput)*.9);
                //changing the target to modulate speed is a TERRIBLE idea. use
                //power or even better max speed
            }
            lastSlideInput = slideInput;
            
            // we fixed the wires, this is no longer neccecary. overide is
            // now just for the arm swing.
             
            if (slideTarget > 1350 && armTarget > 26 && !slideOverride) { //limit before override
                slideTarget = 1350; //This number is how much the arm can extend while down.
                //slide limit ^^ extension limit
                //sadly we had to loose some lenth for the hang. hang extends 5 inch backwards ):
            
            }
            
            if (slideTarget > 4200) { //limit after override, this # could be wrong.
                slideTarget = 4200; //This number is how much the arm can extend.
            }
            
            if (slideTarget < 0) {
                slideTarget = 0;
            }
            
            /*
            * not needed due to hardware rework vvv
            */
            // if (slideTarget < 500 && armTarget > 600 && !slideOverride) { //prevents slide getting pulled in under stuff and arm damaging it.
            //     slideTarget = 500;
            // }
            
            
            //end of manual stuff
        }
        
        
        
        
        // if (grip){
        //     leftGripper.setPosition(0.35);
        //     rightGripper.setPosition(0.63);
        // } else {
        //     leftGripper.setPosition(0.20);
        //     rightGripper.setPosition(0.79);
        // }
        
        
        //readystep for arm procedure. must come before all arm procedures are called each tick.
        if (arm_state != "manual"){
            armReady = (Math.abs(rightArm.getCurrentPosition()- armTarget*armEncoderMultiplyer))<5;
            slideReady = (Math.abs(rightSlide.getCurrentPosition() - slideTarget))<5;
            
            if (armReady && slideReady && readyStep){
            step += 1;
            }
            
        }
        
        if (arm_state == "hang"){
            autoHang();
        }
        
        if (arm_state == "specimine_high"){
            auto_specimine_high();
        }
        
        if (arm_state == "high_basket"){
            auto_high_basket();
        }
        
        
        telemetry.addData("slideOverride", slideOverride);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("slideTarget", slideTarget);
        
        
        //set servo possition
        if (servo_state == "grip"){
            leftGripper.setPosition(0.20);
            rightGripper.setPosition(0.79);
        } else if (servo_state == "release"){
            leftGripper.setPosition(0.4);
            rightGripper.setPosition(0.6);
            // leftGripper.setPosition(0.3);
            // rightGripper.setPosition(0.7);
        } else if (servo_state == "flat"){
            leftGripper.setPosition(0.6);
            rightGripper.setPosition(0.4);
        } else if (servo_state == "specimine_release"){
            leftGripper.setPosition(0.3);
            rightGripper.setPosition(0.7);
        }
        
        //acctually send targets to the motors
        leftSlide.setTargetPosition(slideTarget);
        rightSlide.setTargetPosition(slideTarget);
        
        //we changed from a 30:100 gear ratio to a 20:100, so this
        //is the fix.
        leftArm.setTargetPosition((int) Math.round(armTarget*armEncoderMultiplyer));
        rightArm.setTargetPosition((int) Math.round(armTarget*armEncoderMultiplyer));
        
        
        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);
        
        
        leftArm.setPower(armPower);
        rightArm.setPower(armPower);
        
        
        
        
        
        
        //IMU stuff
        YawPitchRollAngles orientation = leftIMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = leftIMU.getRobotAngularVelocity(AngleUnit.DEGREES);

        // telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        // telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        // telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
        pitch = orientation.getRoll(AngleUnit.DEGREES);
        
        telemetry.addData("servo_state", servo_state);
    
        
        // telemetry.addData("Pitch","guh");
        
        //if a step in an automated procedure has been compleated for the arm, advance!
        
        
        
        
        
        //telemetry (info on the console for debuging)

        telemetry.addData("arm_state",arm_state);
        telemetry.addData("drive_state",drive_state);
        
        telemetry.addData("Step", step);
        telemetry.addData("armReady", armReady);
        telemetry.addData("slideReady", slideReady);
        
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
    
    private double timer_end = 0;
    
    private void auto_high_basket(){
        slidePower = 1;
        armPower = 1;
        
        if (step == -1){ //setup step
            servo_state = "grip"; //should be true or you aint holding anything.
            readyStep = true;
            hangTime1 = 0;
        } else if (step == 0){
            armTarget = 0;
            // slideTarget = 0; sometimes caused the procedure to get stuck
            //as the arm could not go all the way down, consuming power and
            // not advancing.
        } else if (step == 1){
            slideTarget = 4200;
        } else if (step == 2){
            armTarget = 4.17;
        } else if (step == 3){
            readyStep = false;
            if (gamepad2.y){
                servo_state = "release";
                step = 4;
                timer_end = runtime.time()+0.5;
            }
        } else if (step == 4){
            if (runtime.time()>timer_end){
                servo_state = "grip";
                readyStep = true;
            }
        } else if (step == 5){
            armPower = 0.3; //to prevent it from swinging back towards the basket - //doesn't quite work
            armTarget = -3.00;
        } else if (step == 6){
            slideTarget = 0;
            state_machine_clear_arm();
        }
        //
    }
    
    //code to hang
    
    //there is some weird stuff in java were it parses throught here. this is like.. var and not let.. 
    //so all variables defined like this are delcared at the top and not in some weird order.
    
    private void auto_specimine_high(){
        armPower = 1;
        slidePower = 1;
        double TimerStart = 0;
        
        //anything before the if statements is called every tick.
        if (step == -1){
            readyStep = true;
        } else if (step == 0){
            armTarget = 13;//6.78;
            slideTarget = 870;//780;
            readyStep = false;
            if (gamepad2.y){
                readyStep = true;
            }
        } else if (step == 1){
            armPower = 0.7;
            armTarget = 23.47;
        } else if (step == 2){
            armPower = 1;
            readyStep = false;
            if (gamepad2.y){
                servo_state = "specimine_release";
                TimerStart = runtime.time();
                step = 3;
            }
        } else if (step == 3){
            if (runtime.time() > TimerStart + 0.1){
                slideTarget = 400;
                state_machine_clear_arm();
            }
        }
    }
    
    private void autoHang(){
        //this is so the gripper is open and does not interfere with the hang.
        servo_state = "flat";
        //allows the arm and slide to extend past normal limits for the hang.
        slideOverride = true;
        armPower = 1;
        slidePower = 1;
        
        if (step == -1){ //setup step
            readyStep = true;
            hangTime1 = 0;
        } else if (step == 0){
            armTarget = 0;
        } else if (step == 1){
            slideTarget = 2500;
        } else if (step == 2) {
            armTarget = -30; //was -450
        } else if (step == 3) {
            slidePower = 0.5;
            slideTarget = 50;
        } else if (step == 4) {
            slidePower = 1; //new
            armTarget = 15.65;//was 200
        } else if (step == 5) {
            slideTarget = 3600;//new //new new
            step = 6;
        } else if (step == 6) {
            armTarget = 7.82;
            // slideTarget = 3500;
        } else if (step == 7) {
            readyStep = false;
            armTarget = 23.48; //This is the swing back. was 525
            if (leftArm.getCurrentPosition() >= armTarget*armEncoderMultiplyer || rightArm.getCurrentPosition() >= armTarget*armEncoderMultiplyer){
                step = 8;
            }
             //somtimes step doesn't move on when it needs to, thats why
             //we also have pitch thing here.
        } else if (step == 8){
            readyStep = false; //stop using encoders for next step
            armPower = 1;
            armTarget = 0;
            if (pitch<=5.0){
                step = 9;
            }
        } else if (step == 9){
            slidePower = 1;
            slideTarget = 2000; //was 3000 for first grab
            readyStep = true;
        } else if (step == 10){
            armTarget = 90;//46.96;//uhhh
            slideTarget = 1500;
        } else if (step == 11){
            slideTarget = 0;
        } else if (step == 12){
             //state_machine_clear_arm();
        } 
        
       
        
        //autohang
        //remember, isBusy() is a function not a property or variable.
        //make sure a boolean comes out of this - getting any other datatype
        //returns true. be carefull.
        
        
        
        //we were having issues because the arm motor doing the sensing was not
        //meshing with bevel gear. rahh.
        
        /*
         * The code commented out bellow allows you to manualy skip a step. It's been nothing but unhelpfull so I commented it out. only ever breaks stuff.
        */ 
        
        // if (gamepad1.left_trigger > 0.1){
        //     step += 1;
        // }
    }
    //call when an auto arm procedure has finnised to clean up the variables or when you want to exit the procedure.
    private void state_machine_clear_arm(){
        arm_state = "manual"; //gives controll back to user
        readyStep = false;
        step = -1;
        
        slidePower = 1;
        armPower = 1;
    }
}
