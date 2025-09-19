package org.firstinspires.ftc.teamcode.IntoTheDeep;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="DemoTele2", group="Iterative OpMode")

public class DemoTele2 extends OpMode
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
    
    private IMU leftIMU = null;
    
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    
    private boolean grip = false;
    private boolean toggle = false;

    private int armTarget = 0;
    private int slideTarget = 0;
    private int armSpeed = 10;
    private int slideSpeed = 10;
    
    private boolean slideOverride = false;
    
    private boolean armReady = false;
    private boolean slideReady = false;
    
    private double armPower = 1;
    private double slidePower = 1;
    
    private IMU.Parameters IMUparameters;
    private double pitch = 0.0;
    private Map<String, Integer> toggles = new HashMap<>();
    
    //autoarm variables
    
    private int step = -1;
    private boolean readyStep = true;
    private double hangTime1 = 0;
    
    private boolean autoHang = false;
    private boolean armAuto = false;
    private boolean autoHighBasket = false;
    
    
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
        
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        
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
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        
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
        
        //servo controll
        
        if (gamepad2.y){
            if (toggle){
                grip = !grip;
                toggle = false;
            }
        } else {
            toggle = true;
        }
        
        if (grip){
        leftGripper.setPosition(0.35);
        rightGripper.setPosition(0.63);
        } else {
        leftGripper.setPosition(0.20);
        rightGripper.setPosition(0.79);
        }
        
        //controll for drive
        
        double forward = -gamepad1.left_stick_y;
        double Strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        //slowmode
        
        double slowdown = 1-(gamepad1.left_trigger*.75);
        
        
        forward = forward*slowdown;
        Strafe = Strafe*slowdown;
        turn = turn*slowdown;
        
        
        //setpower for drive
        leftFront.setPower(forward + Strafe + turn);
        leftBack.setPower(forward - (Strafe - turn));
        rightFront.setPower(forward - (Strafe + turn));
        rightBack.setPower(forward + (Strafe - turn));
        
        //controll for the arms
        
        //arm controll, but not while hanging. these restrictions will break hang,
        //keep all arm controll within this if so hand is not interfered with.
        if (!armAuto){
            
            //gets gamepad input
            double slideInput = -gamepad2.left_stick_y;
            double armInput = gamepad2.right_stick_y;
            
        //slowmode for arm swing
            
            //slowmode is forced if you are extended too far to prevent gear slippage
            //note slipping can still occur, it's an underlying hardware issue.
            //its between the big and small gear. 12/5/2024
            if (gamepad2.left_bumper || slideTarget > 500){
                armSpeed = 100;
            } else {
                armSpeed = 300;
            }
        //slowmode for slide. no overide for this one (:
            if (gamepad2.left_bumper){
                slideSpeed = 100;
            } else {
                slideSpeed = 300;
            }
            
            //integrate gamepad input to target position, don't let it go
            //into places it should not with limits.
            
            //this is where armtarget is set durring tele
            if (Math.abs(armInput) > 0) {
                armTarget = (int) Math.floor(rightArm.getCurrentPosition()+armInput*armSpeed);
            }
            
            if (armTarget > 1725) {
                armTarget = 1725;
            }
            if (armTarget < 0 && !slideOverride) {
                armTarget = 0;
            }
            if (armTarget < -1500) {
                armTarget = -1500;
            }
            if (slideTarget < 400 && armTarget > 775) { //prevents arm from shoving down if there would be collisions
                armTarget = 775;
                slideTarget = 400;
            }
            
            
            
            //slide controll
            
            //slide overide? (will disconnect wires for hang)
            if(gamepad2.right_trigger > 0.1) {
                slideOverride = true;
            }
            if(gamepad2.left_trigger > 0.1) {
                slideOverride = false;
            }
            
            if (Math.abs(slideInput) > 0) {
                slideTarget = (int) Math.floor(rightSlide.getCurrentPosition()+slideInput*slideSpeed);
            }
            
            // we fixed the wires, this is no longer neccecary. overide is
            // now just for the arm swing.
             
            if (slideTarget > 1900 && armTarget > 500 && !slideOverride) { //limit before override
                slideTarget = 1900; //This number is how much the arm can extend.
                //slide limit ^^
                //sadly we had to loose some lenth for the hang. hang extends 5 inch backwards ):
            }
            
            if (slideTarget > 4200) { //limit after override
                slideTarget = 4200; //This number is how much the arm can extend.
            }
            
            if (slideTarget < 0) {
                slideTarget = 0;
            }
            if (slideTarget < 500 && armTarget > 600 && !slideOverride) { //prevents slide getting pulled in under stuff and arm damaging it.
                slideTarget = 500;
            }
            
                //hang
            if (gamepad1.x){
                autoHang();
                armAuto = true;
                autoHang = true;
            }
            
                //high basket 
            if (gamepad2.x){
                autoHighBasket();
                armAuto = true;
                autoHighBasket = true;
            }
        }
        
        telemetry.addData("slideOverride", slideOverride);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("slideTarget", slideTarget);
        
        
        //acctually send targets to the motors
        leftSlide.setTargetPosition(slideTarget);
        rightSlide.setTargetPosition(slideTarget);
        
        //we changed from a 30:100 gear ratio to a 20:100, so this
        //is the fix.
        leftArm.setTargetPosition(armTarget);
        rightArm.setTargetPosition(armTarget);
        
        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);
        
        leftArm.setPower(armPower);
        rightArm.setPower(armPower);
        
        //autohang
    
        //remember, isBusy() is a function not a property or variable.
        
        if (armAuto){
            if (autoHang) {
                autoHang();
            } else if (autoHighBasket){
                autoHighBasket();
            }
            //make sure a boolean comes out of this - getting any other datatype
            //returns true. be carefull.
            armReady = (Math.abs(rightArm.getCurrentPosition()- armTarget))<5;
            slideReady = (Math.abs(rightSlide.getCurrentPosition() - slideTarget))<5;
            
            // I put armtarget instead of slide...                      ^^^ big problem
            //happened.
            
            //we were having issues because the arm motor doing the sensing was not
            //meshing with bevel gear. rahh.
            
            if (armReady && slideReady && readyStep){
                step += 1;
            }
            
            
            //this is the overide it just breaks everything
            // if (gamepad1.left_trigger > 0.1){
            //     step += 1;
            // }
            
            telemetry.addData("Autohang", autoHang);
            telemetry.addData("AutoHighBasket", autoHighBasket);
            telemetry.addData("ArmAuto", armAuto);
            
            telemetry.addData("Autohang Step", step);
            telemetry.addData("armReady", armReady);
            telemetry.addData("slideReady", slideReady);
        }
        
        
        //IMU stuff
        YawPitchRollAngles orientation = leftIMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = leftIMU.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
        pitch = orientation.getRoll(AngleUnit.DEGREES);
        
        telemetry.addData("Pitch","guh");
        
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
    
    //code to hang
    
    
    
    private void autoHang(){
        step = -1;
        readyStep = true;
        hangTime1 = 0;
        
        //this is so the gripper is open and does not interfere with the hang.
        grip = false;
        //allows the arm and slide to extend past normal limits for the hang.
        slideOverride = true;
        
        
        if ((gamepad1.right_trigger > 0.1)&&(gamepad1.left_trigger > 0.1)){
            armAuto = false; //gives controll back to user
            autoHang = false;
            readyStep = false; //stops the thing from going to infinity steps
            //also, for your manual overide, REMEMBER TO PRESS "A"!
        }else if (step == -1){
            armTarget = 0;
        } else if (step == 0){
            slideTarget = 2500;
        } else if (step == 1) {
            armTarget = -450;
        } else if (step == 2) {
            slidePower = 0.5;
            slideTarget = 50;
        } else if (step == 3) {
            armTarget = 200;
        } else if (step == 4) {
            slideTarget = 1000;
        } else if (step == 5) {
            armTarget = 150;
            slideTarget = 3500;
        } else if (step == 6) {
            armTarget = 525;
            if (leftArm.getCurrentPosition() >= armTarget || rightArm.getCurrentPosition() >= armTarget){
                step = 7;
            }
             //somtimes step doesn't move on when it needs to, thats why
             //we also have pitch thing here.
        } else if (step == 7){
            readyStep = false; //stop using encoders for next step
            armPower = 1;
            armTarget = 0;
            if (pitch<=5.0){
                step = 8;
            }
        } else if (step == 8){
            slidePower = 1;
            slideTarget = 2000; //was 3000 for first grab
            readyStep = true;
        } else if (step == 9){
            armTarget = 1000;
            slideTarget = 1500;
        } else if (step == 10){
            slideTarget = 0;
        }
    }
    
    private void autoHighBasket(){
        step = -1;
        readyStep = true;
        
        //this is so the gripper is open and does not interfere with the hang.
        //allows the arm and slide to extend past normal limits for the hang.
        slideOverride = true;
        
        
        if ((gamepad1.right_trigger > 0.1)&&(gamepad1.left_trigger > 0.1)){
            armAuto = false; //gives controll back to user
            autoHighBasket = false;
            readyStep = false; //stops the thing from going to infinity steps
            //also, for your manual overide, REMEMBER TO PRESS "A"!
        }else if (step == -1){
            armTarget = 0;
        } else if (step == 0){
            slideTarget = 4200;
        } else if (step == 1) {
            armTarget = 150;
        } else if (step == 2) {
           grip = false;
        } else if (step == 3) {
            armTarget = 0;
        } else if (step == 4) {
            slideTarget = 0;
        } else if (step == 5) {
            armAuto = false;
            autoHighBasket = false;
            slideOverride = false;
        }
    }
}
