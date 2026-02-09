package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="SwerveTest")
public class SwerveTest extends OpMode {


    ///swerve related mechanical constants
    double Encoder_resolution = 537.7;

    //gear ratio constants
    double motor_gear_teeth = 15;
    double large_diff_spur_teeth = 66;
    double large_diff_bevel_teeth = 64;
    double small_diff_bevel_teeth = 18;

    ///swerves
    SwervePod leftSwerve;
    SwervePod rightSwerve;

    ///swerve class definition VV
    public class SwervePod {

        ///Optional or can't provide at start VV
        double heading = 0;
        double start_rotation = -0.25; //If your pod is starting facing forwards, use this. 1 "turn" = 360 degrees
        double center_x_offset = 0;
        double center_y_offset = 0;

        public PID heading_pid = new PID(5,0.4,0);

        ///Must provide to work VV

        //motors
        double Encoder_resolution; //ticks per revolution
        private DcMotorEx up_motor;
        private DcMotorEx down_motor;

        //gears
        double motor_gear_teeth = 15;
        double large_diff_spur_teeth = 66;
        double large_diff_bevel_teeth = 64;
        double small_diff_bevel_teeth = 18;

        double rotation_encoder_ratio;

        /// prohibited from being altered
        private double heading_target_jumps = 0; //This will be increments of 0.5. could be an int but im lazy with type conversions.
        double target_heading = 0;

        ///Constructor(s)

        //create during init() -- pretty please
        public SwervePod(
            double Encoder_resolution,
            DcMotorEx up_motor,
            DcMotorEx down_motor,
            double motor_gear_teeth,
            double large_diff_spur_teeth,
            double large_diff_bevel_teeth,
            double small_diff_bevel_teeth
        ){
            //rotation (yaw) will not work if you do not provide a center

            this.Encoder_resolution = Encoder_resolution;
            this.up_motor = up_motor;
            this.down_motor = down_motor;
            this.motor_gear_teeth = motor_gear_teeth;
            this.large_diff_spur_teeth = large_diff_spur_teeth;
            this.large_diff_bevel_teeth = large_diff_bevel_teeth;
            this.small_diff_bevel_teeth = small_diff_bevel_teeth;
            this.rotation_encoder_ratio = (motor_gear_teeth/large_diff_spur_teeth)/(Encoder_resolution);
            this.target_heading = this.start_rotation;

            up_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            down_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            up_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            down_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        ///Methods

        public void setHeadingZero(){
            up_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            down_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            down_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void updatePod(double translational_x,double translational_y, double yaw){
            //1 Swerve pod code

            //gear math operations
            double r_average_encoder_position = (up_motor.getCurrentPosition()+ down_motor.getCurrentPosition())/2;

            heading = start_rotation + r_average_encoder_position* rotation_encoder_ratio; //This is in 360 turns, not radians.

            if ((Math.abs(translational_y)+Math.abs(translational_x))>0) { //Math.pow and sqrt for length but we don't need that so this is easier on the computer
                target_heading = Math.atan2(translational_y,translational_x)/(Math.PI*2);
            }

            //avoids turning more than we need. Should ensure forward always faces where we want! 0.5 means the wheel allways faces forward, with 0.25 we can reverse the wheel and it can be w
            boolean condition_greater = true;
            while (condition_greater){
                condition_greater = (heading -(target_heading + heading_target_jumps))>0.25;
                if (condition_greater){
                    heading_target_jumps += 0.5;
                }
            }
            boolean condition_lesser = true;
            while (condition_lesser){
                condition_lesser = (heading -(target_heading + heading_target_jumps))<-0.25;
                if (condition_lesser){
                    heading_target_jumps -= 0.5;
                }
                telemetry.addData("changing this!","true");
            }

            boolean drive_flip; //is the drive wheel 180 the wrong way and we need to drive reverse?

            if (Math.abs(heading_target_jumps) != Math.floor(Math.abs(heading_target_jumps))){
                drive_flip = true;
            } else {
                drive_flip = false;
            }
            telemetry.addData("flip: ",drive_flip);

            double turn = heading_pid.update(target_heading + heading_target_jumps, heading); //This is in 360 turns, not radians.
             //clockwise/right +
            telemetry.addData("l_desired_rotation_turns", target_heading);
            telemetry.addData("l_rotation_target_jumps", heading_target_jumps);

            double forward = gamepad1.right_trigger-gamepad1.left_trigger;

            double up_power = -forward+turn;
            double down_power = forward+turn;

            up_motor.setPower(up_power);
            down_motor.setPower(down_power);
            telemetry.addData("Top MotorPower:",up_power);
            telemetry.addData("Bottom MotorPower:",down_power);
            telemetry.addData("pod rotation: ", heading);
        }
    }

    //End of swerve definition related things
    //
    //
    /// Opmode stuff!

    @Override public void init(){


        leftSwerve = new SwervePod(
                Encoder_resolution,
                hardwareMap.get(DcMotorEx.class,"LSwerveUp"),
                hardwareMap.get(DcMotorEx.class,"LSwerveDown"),
                motor_gear_teeth,
                large_diff_spur_teeth,
                small_diff_bevel_teeth,
                small_diff_bevel_teeth
        );

        rightSwerve = new SwervePod(
                Encoder_resolution,
                hardwareMap.get(DcMotorEx.class,"RSwerveUp"),
                hardwareMap.get(DcMotorEx.class,"RSwerveDown"),
                motor_gear_teeth,
                large_diff_spur_teeth,
                small_diff_bevel_teeth,
                small_diff_bevel_teeth
        );

    }

    @Override public void init_loop(){
        if (gamepad1.xWasPressed()){
            leftSwerve.setHeadingZero();
            rightSwerve.setHeadingZero();
        }
        //leftSwerve.updatePod(0,0,0);

        //don't do this ^^ because zero power breaking? unsure. TBD later.
    }

    @Override public void loop(){
        //translation and rotation desires
        double translational_x = gamepad1.left_stick_x;
        double translational_y = gamepad1.left_stick_y;

        double yaw = gamepad1.right_stick_x;

        leftSwerve.updatePod(translational_x,translational_y,yaw);
        rightSwerve.updatePod(translational_x,translational_y,yaw);
    }
}
