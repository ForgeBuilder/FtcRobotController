package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ShooterTest", group="Iterative OpMode")

public class ShooterTest extends OpMode
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
    
    @Override
    public void init() {
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");


    }

    private boolean shooting = false;
    
    @Override
    public void loop() {
        if (gamepad1.xWasPressed()){
            shooting = !shooting;
        }
        if (shooting){
            rightArm.setPower(1);
        } else{
            rightArm.setPower(0);
        }


    }
}