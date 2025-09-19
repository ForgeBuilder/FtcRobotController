package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DemoTele", group="Iterative OpMode")

public class DemoTele extends OpMode
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
    
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    
    private boolean grip = false;
    private boolean toggle = false;
    
    private boolean raise = false;
    private boolean toggle2 = false;
    
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    
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
    
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        
        leftArm.setDirection(DcMotor.Direction.FORWARD);
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
        
        //make arms in runtoposition for demo
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        rightArm.setTargetPosition(0);
        leftArm.setTargetPosition(0);
        
        rightArm.setPower(1);
        leftArm.setPower(1);
        
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // make the right side wheels reversed
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
        
        if (gamepad2.y || gamepad1.y){
            if (toggle){
                grip = !grip;
                toggle = false;
            }
        } else {
            toggle = true;
        }
        
        if (grip){
        leftGripper.setPosition(0.20);
        rightGripper.setPosition(0.79);
        } else {
        leftGripper.setPosition(0.5);
        rightGripper.setPosition(0.5);   
        }
        
        //raise arm or lower for demo mode.
        
        if (gamepad2.x || gamepad1.x){
            if (toggle2){
                raise = !raise;
                toggle2 = false;
                
                if (raise){
                    
                    rightArm.setTargetPosition(-2000);
                    leftArm.setTargetPosition(-2000);

                } else {
                    
                    rightArm.setTargetPosition(0);
                    leftArm.setTargetPosition(0);
                }
            } 
        } else {
                toggle2 = true;
        }
        
        //controll for drive
        
        double forward = -gamepad1.left_stick_y;
        double Strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        //slowmode
        
        if (true){ //should be (gamepad1.left_bumper), permenant for demo
         forward = forward/3;
         Strafe = Strafe/3;
         turn = turn/3;
        }
        
        //setpower for drive
        leftFront.setPower(forward + Strafe + turn);
        leftBack.setPower(forward - (Strafe - turn));
        rightFront.setPower(forward - (Strafe + turn));
        rightBack.setPower(forward + (Strafe - turn));
        
        //controll for the arms - disabled because demomode
        
        // double slidePower = gamepad2.left_stick_y;
        // double armPower = gamepad2.right_stick_y;
      
        // if (gamepad2.left_bumper){
        //  armPower = armPower/3;
        // }
      
        // leftSlide.setPower(slidePower);
        // rightSlide.setPower(slidePower);
        
        // leftArm.setPower(armPower);
        // rightArm.setPower(armPower);
        
        // Show the elapsed game time and wheel power.
        
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Toggle2", toggle2);
        telemetry.addData("raise",raise);
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
