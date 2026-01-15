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

        //append to telemetry without clearing previous
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.setAutoClear(true);

        super.loop();
    }
}
