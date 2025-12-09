package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Array;

import dalvik.system.DelegateLastClassLoader;

@Configurable
@TeleOp(name="LauncherPIDtuner")
public class Crossbow_launcher_PID_Tuner extends CrossbowTeleop{

    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(0.0,0.0,0.0,0.0);
    private int selector = 0;

    private DcMotorEx leftLaunchMotor;
    private DcMotorEx rightLaunchMotor;

    public static Double[] PIDFCoefficientsList = {0.0,0.0,0.0,0.0};

    @Override
    public void init(){
        super.init();
        leftLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm2");
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class,"lm1");
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
    }

    @Override
    public void loop(){
        super.loop();

        telemetry.addData("p",PIDFCoefficientsList[0]);
//        if (selector == 0){telemetry.addData("^","");}
        telemetry.addData("i",PIDFCoefficientsList[1]);
//        if (selector == 1){telemetry.addData("^","");}
        telemetry.addData("d",PIDFCoefficientsList[2]);
//        if (selector == 2){telemetry.addData("^","");}
        telemetry.addData("f",PIDFCoefficientsList[3]);
//        if (selector == 3){telemetry.addData("^","");}
        if (gamepad1.xWasPressed()){
            launcherCoefficients = new PIDFCoefficients(PIDFCoefficientsList[0],PIDFCoefficientsList[1],PIDFCoefficientsList[2],PIDFCoefficientsList[3]);
            leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
            rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
        }

        if (follower.isBusy()) {
            if (gamepad1.x){
                follower.breakFollowing();
            }
        } else {
            //this is a little nonsensical. I might as well have just put all the teleop functions in here
            //and made the motors public. It is what it is.. this is how we learn!

            //For the teleop functions I could just have them in here and give them refrences to what they need.
            drive_with_teleop(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.left_trigger,
                    ((gamepad1.right_trigger > 0.1)||(gamepad2.right_trigger > 0.1))
            );
        }

        //append to telemetry without clearing previous
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.setAutoClear(true);

        panelsTelemetry.update(telemetry);
    }


    private boolean spin_intake = false;
    private ElapsedTime intake_reverse_timer = new ElapsedTime();
    public void intake_code(){
        if (gamepad2.aWasPressed()||gamepad1.leftBumperWasPressed()){
            spin_intake = !spin_intake;
        }
        //it's a 312 so 537.7 PPR at the Output Shaft. 5.2 RPS (max) would be 2796.04 or about 2800.
        if ((spin_intake||trying_to_fire)&&(!kick)){
            set_intake_speed(2000);
        } else if(gamepad2.aWasReleased()||gamepad1.leftBumperWasReleased()) {
            intake_reverse_timer.reset();
        } else if((gamepad2.a||gamepad1.left_bumper)&&intake_reverse_timer.seconds()>0.5) {
            set_intake_speed(-2000);
        } else {
            set_intake_speed(0);
        }
    }
}
