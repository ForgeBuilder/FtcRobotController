package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DrivetrainTest")
public class DrivetrainTest extends OpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    @Override
    public void init(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
    }

    @Override
    public void loop(){
        double leftFrontPower = gamepad1.left_stick_y;
        double rightFrontPower = gamepad1.right_stick_y;
        double leftBackPower = gamepad2.left_stick_y;
        double rightBackPower = gamepad2.right_stick_y;


        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("leftFrontPower",leftFrontPower);
        telemetry.addData("rightFrontPower",rightFrontPower);
        telemetry.addData("leftBackPower",leftBackPower);
        telemetry.addData("rightBackPower",rightBackPower);
    }
}
