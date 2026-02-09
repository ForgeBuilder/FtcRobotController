package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

@TeleOp(name="SwerveTest")
public class SwerveTest extends OpMode {

    double l_rotation = 0;
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



        r_swerve_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_swerve_down.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static PID l_rotation_pid = new PID(5,0.2,0);

    private double l_rotation_target_jumps = 0; //This will be increments of 0.5. could be an int but im lazy with type conversions.

    double l_desired_rotation_turns = 0;

    @Override public void loop(){

        if (gamepad1.xWasPressed()){
//            reset encoders
        l_swerve_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_swerve_down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_swerve_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_swerve_down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        r_swerve_up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_swerve_down.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_swerve_up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_swerve_down.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        //gear math
        double r_average_encoder_position = (l_swerve_up.getCurrentPosition()+ l_swerve_down.getCurrentPosition())/2;
        double r_rotation_encoder_ratio = (motor_gear_teeth/large_diff_bevel_teeth)/(Encoder_resolution);


        //translation and rotation desires
        double translational_x = gamepad1.left_stick_x;
        double translational_y = gamepad1.left_stick_y;

        double yaw = gamepad1.right_stick_x;

        double start_rotation = -0.25;

        //1 Swerve pod code

        l_rotation = start_rotation + r_average_encoder_position*r_rotation_encoder_ratio; //This is in 360 turns, not radians.

        double l_forward = 0;

        if ((Math.abs(translational_y)+Math.abs(translational_x))>0){ //Math.pow and sqrt for length but we don't need that so this is easier on the computer
            l_desired_rotation_turns = Math.atan2(translational_y,translational_x)/(Math.PI*2);
        } //else do nothing

        //avoids turning more than we need. Should ensure forward always faces where we want! 0.5 means the wheel allways faces forward, with 0.25 we can reverse the wheel and it can be w
        boolean condition_greater = true;
        boolean condition_lesser = true;
        while (condition_greater || condition_lesser){
             condition_greater = (l_rotation-(l_desired_rotation_turns+l_rotation_target_jumps))>0.25;
             condition_lesser = (l_rotation-(l_desired_rotation_turns+l_rotation_target_jumps))<-0.25;

             if (condition_greater){
                 l_rotation_target_jumps += 0.5;
             }
             if (condition_lesser){
                 l_rotation_target_jumps -= 0.5;
             }
        }

        boolean drive_flip; //is the drive wheel 180 the wrong way and we need to drive reverse?

        if (Math.abs(l_rotation_target_jumps)/2 != Math.floor(Math.abs(l_rotation_target_jumps)/2)){
            drive_flip = true;
        } else {
            drive_flip = false;
        }
        telemetry.addData("flip: ",drive_flip);




        double l_turn = l_rotation_pid.update(l_desired_rotation_turns+l_rotation_target_jumps,l_rotation); //This is in 360 turns, not radians.
        ; //clockwise/right +

        double l_swerve_up_power = -l_forward+l_turn;
        double l_swerve_down_power = l_forward+l_turn;

        l_swerve_up.setPower(l_swerve_up_power);
        l_swerve_down.setPower(l_swerve_down_power);
        telemetry.addData("Top MotorPower:",l_swerve_up_power);
        telemetry.addData("Bottom MotorPower:",l_swerve_down_power);



        telemetry.addData("pod rotation: ", l_rotation);



//
//        double r_forward = 0;
//        double r_turn = 0; //clockwise/right +
//
//        r_swerve_up.setPower(-r_forward+r_turn);
//        r_swerve_down.setPower(r_forward+r_turn);
    }
}
