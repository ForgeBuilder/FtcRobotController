package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.sql.Array;


@TeleOp(name="LauncherPIDtuner")
public class Crossbow_launcher_PID_Tuner extends CrossbowTeleop{

    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(250,2,0,0);
    private int selector = 0;

    private DcMotorEx leftLaunchMotor;
    private DcMotorEx rightLaunchMotor;

    private Integer[] PIDFCoefficientsList = {0,0,0,0};

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
            PIDFCoefficientsList[selector] += 1;
        }
        if (gamepad1.dpadDownWasPressed()){
            PIDFCoefficientsList[selector] -= 1;
        }
        telemetry.addData("p",PIDFCoefficientsList[0]);
        telemetry.addData("i",PIDFCoefficientsList[1]);
        telemetry.addData("d",PIDFCoefficientsList[2]);
        telemetry.addData("f",PIDFCoefficientsList[3]);
        if (gamepad1.xWasPressed()){
            launcherCoefficients = new PIDFCoefficients(PIDFCoefficientsList[0],PIDFCoefficientsList[1],PIDFCoefficientsList[2],PIDFCoefficientsList[3]);
            leftLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
            rightLaunchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,launcherCoefficients);
        }
    }
}
