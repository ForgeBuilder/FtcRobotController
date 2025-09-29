package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="TeleopMain")

public class TeleopMain extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialise motor variables

    private DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
    private DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");

    private DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
    private DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront.setDirection(DcMotor.Direction.REVERSE);

        //Make every motor break when at power 0
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        if (true) {
            //manual controll for drive

            double forward = -gamepad1.left_stick_y;
            double Strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //slide recalibrate..

            //power slides so they retract and stop/reset encoders with the top bumpers

            //driver 1's slowmode

            double slowdown = 1 - (gamepad1.left_trigger * .75);


            forward = forward * slowdown;
            Strafe = Strafe * slowdown;
            turn = turn * slowdown;


            //setpower for drive
            leftFront.setPower(forward + Strafe + turn);
            leftBack.setPower(forward - (Strafe - turn));
            rightFront.setPower(forward - (Strafe + turn));
            rightBack.setPower(forward + (Strafe - turn));
        }

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
}