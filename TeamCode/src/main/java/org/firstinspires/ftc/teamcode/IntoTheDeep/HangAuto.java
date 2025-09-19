package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="HangAuto", group="Linear OpMode")

public class HangAuto extends LinearOpMode {

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor rightSlide = null;
    private DcMotor rightArm = null;
    
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor leftSlide = null;
    private DcMotor leftArm = null;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
    
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
    
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        
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
        
        // make the right side wheels reversed
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        telemetry.addData("Status", "Active");
        telemetry.update();
        
        // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // leftBack.setTargetPosition(-1000);
        // leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftBack.setPower(0.5);
        
        // waitForMotor();
        
        // leftBack.setTargetPosition(0);
        
        // waitForMotor();
        
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //where all the movement is called using premade functions
        
        // SetSlideTargetPosition(-1000);
        // SetArmTargetPosition(1000);
        // SetSlideTargetPosition(-500);
        // SetArmTargetPosition(0);
        // SetSlideTargetPosition(0);
        
        // SetArmTargetPosition(200);
        // SetSlideTargetPosition(-4300);//this is how far the slide can go
        // SetSlideTargetPosition(0);
        // SetArmTargetPosition(0);
        
        SetSlideTargetPosition(-2500);
        // sleep(500);
        SetArmTargetPosition(320);
        // sleep(1000);
        SetSlideTargetPosition(0);
        // sleep(1000);
        SetArmTargetPosition(-200);
        // sleep(1000);
        SetSlideTargetPosition(-2500);
        // sleep(5000);
        SetArmTargetPosition(-900);
        //
        SetArmTargetPosition(0);
        sleep(5000);
        
    }

private void waitForMotor() {
    while(opModeIsActive() && leftBack.isBusy()){
            telemetry.addData("Status", "Idle");
            telemetry.update();
            idle();
        }
}

private void SetArmTargetPosition(int target) {
    
    rightArm.setTargetPosition(target);
    leftArm.setTargetPosition(target);
    
    rightArm.setPower(1);
    leftArm.setPower(1);
    
    leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    while(opModeIsActive() && leftArm.isBusy()&&rightArm.isBusy()){
            telemetry.addData("Status", "Idle");
            telemetry.update();
            idle();
        }
}

private void SetSlideTargetPosition(int target) {
    
    rightSlide.setTargetPosition(target);
    leftSlide.setTargetPosition(target);
    
    rightSlide.setPower(.5);
    leftSlide.setPower(.5);
    
    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    while(opModeIsActive() && leftSlide.isBusy()&&rightSlide.isBusy()){
            telemetry.addData("Status", "Idle");
            telemetry.update();
            idle();
        }
}

}
    
