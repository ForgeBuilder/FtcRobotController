package org.firstinspires.ftc.teamcode;

import static
com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")

public class TeleOpJava extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
    }
    private LaunchState launchState;
    double leftPower;
    double rightPower;
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }
    @Override
    public void loop(){
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    @Override
    public void stop() {
    }
    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}
