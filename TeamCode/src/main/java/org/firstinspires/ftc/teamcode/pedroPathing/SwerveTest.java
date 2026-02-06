package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="SwerveTest")
public class SwerveTest extends OpMode {

    double rotation = 0;
    double Encoder_resolution = 537.7; //ticks per revolution

    //gear ratio constants
    double large_diff_spur_teeth = 66;
    double large_diff_bevel_teeth = 64;
    double motor_gear_teeth = 15;
    double small_diff_bevel_teeth = 18;
    private DcMotorEx swerve1;
    private DcMotorEx swerve2;

    @Override public void init(){

        swerve1 = hardwareMap.get(DcMotorEx.class,"swerve 1");
        swerve2 = hardwareMap.get(DcMotorEx.class,"swerve 2");

        //reset encoders
        swerve1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swerve2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swerve1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swerve2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override public void loop(){
        swerve1.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
        swerve2.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);

        double average_encoder_position = (swerve1.getCurrentPosition()+swerve2.getCurrentPosition())/2;

        double rotation_encoder_ratio = (motor_gear_teeth/large_diff_bevel_teeth)/(Encoder_resolution);

        rotation = average_encoder_position*rotation_encoder_ratio;

        rotation = average_encoder_position*rotation_encoder_ratio;

        telemetry.addData("pod rotation: ",rotation);
    }
}
