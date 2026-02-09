package org.firstinspires.ftc.teamcode.pedroPathing;

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

    //motors delcared in order they are plugged into the controller left to right, control hub to expansion hub
    private DcMotorEx l_swerve_up;
    private DcMotorEx l_swerve_down;
    private DcMotorEx r_swerve_up;
    private DcMotorEx r_swerve_down;


    @Override public void init(){

        l_swerve_up = hardwareMap.get(DcMotorEx.class,"LSwerveUp");
        l_swerve_down = hardwareMap.get(DcMotorEx.class,"LSwerveDown");

        r_swerve_up = hardwareMap.get(DcMotorEx.class,"RSwerveUp");
        r_swerve_down = hardwareMap.get(DcMotorEx.class,"RSwerveDown");




        //reset encoders
        l_swerve_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_swerve_down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_swerve_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_swerve_down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        r_swerve_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_swerve_down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_swerve_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_swerve_down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        r_swerve_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop(){
        double translational_x = gamepad1.left_stick_x;
        double translational_y = gamepad1.left_stick_y;

        double yaw = gamepad1.right_stick_x;



        l_swerve_up.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
        l_swerve_down.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);

        r_swerve_up.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
        r_swerve_down.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);

        double r_average_encoder_position = (l_swerve_up.getCurrentPosition()+ l_swerve_down.getCurrentPosition())/2;
        double r_rotation_encoder_ratio = (motor_gear_teeth/large_diff_bevel_teeth)/(Encoder_resolution);

        rotation = r_average_encoder_position*r_rotation_encoder_ratio;

        rotation = r_average_encoder_position*r_rotation_encoder_ratio;

        telemetry.addData("pod rotation: ",rotation);
    }
}
