package org.firstinspires.ftc.teamcode;

import static
com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")

public class TeleOpJava extends OpMode {
    private DcMotor lfDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor rbDrive = null;
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
        lfDrive = hardwareMap.get(DcMotor.class, "lf");
        lbDrive = hardwareMap.get(DcMotor.class, "lb");
        rfDrive = hardwareMap.get(DcMotor.class, "rf");
        rbDrive = hardwareMap.get(DcMotor.class, "rb");
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        rbDrive.setDirection(DcMotor.Direction.REVERSE);
        lfDrive.setZeroPowerBehavior(BRAKE);
        lbDrive.setZeroPowerBehavior(BRAKE);
        rfDrive.setZeroPowerBehavior(BRAKE);
        rbDrive.setZeroPowerBehavior(BRAKE);

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
        lfDrive.setPower(leftPower);
        lbDrive.setPower(leftPower);
        rfDrive.setPower(rightPower);
        rbDrive.setPower(rightPower);
    }
}
