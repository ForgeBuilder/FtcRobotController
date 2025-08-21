package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive forward)",preselectTeleOp = "TeleopMain")



public class DriveForwardTest extends LinearOpMode {

    /* Declare OpMode members. */
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


    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.2; //change this to max speed instead of power
    static final double     TURN_SPEED    = 0.5;

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
        // leftGripper.setPosition(0);
        // rightGripper.setPosition(1);
        
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
        
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        
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
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
    
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //put servos in air
        // leftGripper.setPosition(0.15);
        // rightGripper.setPosition(0.84);

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for x seconds
        
        
        leftFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        
        int distance = -1050;
        
        leftFront.setTargetPosition(distance);
        leftBack.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);
        rightBack.setTargetPosition(distance);
        
        while(leftFront.isBusy()){
            telemetry.addData("leftFront",leftFront.getCurrentPosition());
            telemetry.addData("rightFront",rightFront.getCurrentPosition());
            telemetry.addData("leftBack",leftBack.getCurrentPosition());
            telemetry.addData("rightBack",rightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Step 4:  Stop

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    // private void driveWithTarget(int leftTarget, int rightTarget){
    //     leftBack.setMode(com.qualcomm.robotcore.hardware.DcMotor);
        
    // }
}
