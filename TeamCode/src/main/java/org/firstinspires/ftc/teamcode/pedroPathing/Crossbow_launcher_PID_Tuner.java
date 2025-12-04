package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Array;

import dalvik.system.DelegateLastClassLoader;


@TeleOp(name="LauncherPIDtuner")
public class Crossbow_launcher_PID_Tuner extends CrossbowTeleop{

    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(0.0,0.0,0.0,0.0);
    private int selector = 0;

    private DcMotorEx leftLaunchMotor;
    private DcMotorEx rightLaunchMotor;

    private Double[] PIDFCoefficientsList = {25.0,20.0,0.0,0.0};

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
        launcher_code(gamepad1.left_trigger > 0.1,gamepad1.left_bumper);
        intake_code();
        if (gamepad1.dpadLeftWasPressed()){
            selector+=1;
            if (selector == 4){
                selector = 0;
            }
        }
        if (gamepad1.dpadRightWasPressed()){
            selector-=1;
            if (selector == -1){
                selector = 3;
            }
        }
        if (gamepad1.dpadUpWasPressed()){
            PIDFCoefficientsList[selector] += 1.0;
        }
        if (gamepad1.dpadDownWasPressed()){
            PIDFCoefficientsList[selector] -= 1.0;
        }
        telemetry.addData("p",PIDFCoefficientsList[0]*10);
        if (selector == 0){telemetry.addData("^","");}
        telemetry.addData("i",PIDFCoefficientsList[1]/10);
        if (selector == 1){telemetry.addData("^","");}
        telemetry.addData("d",PIDFCoefficientsList[2]);
        if (selector == 2){telemetry.addData("^","");}
        telemetry.addData("f",PIDFCoefficientsList[3]);
        if (selector == 3){telemetry.addData("^","");}
        if (gamepad1.xWasPressed()){
            launcherCoefficients = new PIDFCoefficients(PIDFCoefficientsList[0]*10,PIDFCoefficientsList[1]/10,PIDFCoefficientsList[2],PIDFCoefficientsList[3]);
            leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
            rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
        }
        telemetry.update();
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
