package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="BasketAuto)",preselectTeleOp = "TeleopMain")



public class BasketAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private IMU leftIMU = null;
    
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor rightSlide = null;
    private DcMotor rightArm = null;
    
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor leftSlide = null;
    private DcMotor leftArm = null;
    
    //how many encoder ticks are needed for 1 degree of turning?
    private double armEncoderMultiplyer;
    
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    
    private IMU.Parameters IMUparameters;
    double currentyaw = 0.0;

    private ElapsedTime     runtime = new ElapsedTime();
    
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    static final double     FORWARD_SPEED = 0.2; //change this to max speed instead of power
    static final double     TURN_SPEED    = 0.4;

    @Override
    public void runOpMode() {
        
        // Initialize the drive system variables.
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
        
        //stabilise
        leftGripper.setPosition(0.7);
        rightGripper.setPosition(0.4);
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        
        //make everything break - not broken but stop moving when 0
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);
        
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD); 
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
    
        
        
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
        
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //get servos ready to grab
        leftGripper.setPosition(0.4);
        rightGripper.setPosition(0.6);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for x seconds
        
        
        setDriveSpeed(FORWARD_SPEED);
        
        leftArm.setPower(1);
        rightArm.setPower(1);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        
        //reverse from specimine
        leftArm.setTargetPosition(2650);
        rightArm.setTargetPosition(2650);
        
        driveWithTarget(300,300);
        
        setDriveSpeed(0.3);
        
        strafeWithTarget(650);
        
        setDriveSpeed(0.75);
        
        driveWithTarget(750,750);
        
        leftGripper.setPosition(0.20);
        rightGripper.setPosition(0.80);
        sleep(300);
        
        driveWithTarget(-450,-450);
        
        turnWithTarget(-135);
        
        //get into position for low basket
        swingArmWithTarget(550);
        leftSlide.setTargetPosition(2000);
        rightSlide.setTargetPosition(2000);
        while (leftSlide.isBusy()){
            idle();
        }
        
        setDriveSpeed(0.5);
        driveWithTarget(500,500);
        
        leftGripper.setPosition(0.4);
        rightGripper.setPosition(0.6);
        sleep(300);
        swingArmWithTarget(0);
        leftSlide.setTargetPosition(1500);
        rightSlide.setTargetPosition(1500);
        
        //face second sample
        
        turnWithTarget(125);
        swingArmWithTarget(2650);
        driveWithTarget(300,300);
        //grab second sample
        leftGripper.setPosition(0.2);
        rightGripper.setPosition(0.8);
        
        sleep(400);
        swingArmWithTarget(0);
        leftSlide.setTargetPosition(2000);
        rightSlide.setTargetPosition(2000);
        
        driveWithTarget(-300,-300);
        
        turnWithTarget(-125);
        
        swingArmWithTarget(550);
        leftGripper.setPosition(0.4);
        rightGripper.setPosition(0.6);
        sleep(500);
        
        swingArmWithTarget(0);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        while(leftSlide.isBusy()){
            idle();
        }
        
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    private void swingArmWithTarget(int target){
        leftArm.setTargetPosition(target);
        rightArm.setTargetPosition(target);
        
        //make this a function later
        while(leftArm.isBusy()){
            telemetry();
            idle();
        }
    }
    
    private void setDriveSpeed(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    
    private void strafeWithTarget(int target){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftFront.setTargetPosition(-target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(-target);
        
        while(
            leftFront.isBusy()||
            rightFront.isBusy()||
            leftBack.isBusy()||
            rightBack.isBusy()
            ){
            telemetry();
            idle();
        }
        driveMotorReset();
    }
    //notperfect as it does not wait for all to stop being busy.
    
    
    
    private void driveWithTarget(int leftTarget, int rightTarget){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftFront.setTargetPosition(leftTarget);
        leftBack.setTargetPosition(leftTarget);
        rightFront.setTargetPosition(rightTarget);
        rightBack.setTargetPosition(rightTarget);
        
        while(
                leftFront.isBusy()||
                rightFront.isBusy()||
                leftBack.isBusy()||
                rightBack.isBusy()
            ){
            telemetry();
            idle();
        }
        driveMotorReset();
    }
    
    private void turnWithTarget(double bearing){
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        leftIMU.resetYaw();
        currentyaw = 0;
        
        //problem
        
        double turnSpeed = TURN_SPEED*(bearing/Math.abs(bearing));
        
        leftFront.setPower(turnSpeed);
        leftBack.setPower(turnSpeed);
        rightFront.setPower(-turnSpeed);
        rightBack.setPower(-turnSpeed);
        
        while(Math.abs(currentyaw) < Math.abs(bearing)){
            orientation = leftIMU.getRobotYawPitchRollAngles();
            angularVelocity = leftIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            
            currentyaw = orientation.getYaw(AngleUnit.DEGREES);
            telemetry();
            idle();
        }
        driveMotorReset();
    }
    
    private void telemetry(){
        YawPitchRollAngles orientation = leftIMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = leftIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        
        telemetry.addData("leftFront",leftFront.getCurrentPosition());
        telemetry.addData("rightFront",rightFront.getCurrentPosition());
        telemetry.addData("leftBack",leftBack.getCurrentPosition());
        telemetry.addData("rightBack",rightBack.getCurrentPosition());
        telemetry.addData("","");
        telemetry.addData("leftArm",leftArm.getCurrentPosition());
        telemetry.addData("rightArm",rightArm.getCurrentPosition());
        telemetry.addData("leftSlide",leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide",rightSlide.getCurrentPosition());
        
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        
        telemetry.addData("","");
    }
    
    private void driveMotorReset(){
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);
        
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setDriveSpeed(1.0);
    }
}
    
    
