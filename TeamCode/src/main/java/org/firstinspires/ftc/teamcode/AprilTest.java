package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import java.util.ArrayList;


@TeleOp(name="AprilTest", group="Iterative OpMode")

public class AprilTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    // Initialise motor variables
    
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private CRServo intakeServo;

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;


    @Override
    public void init() {
//        rightFront = hardwareMap.get(DcMotor.class, "rf");
//        rightBack = hardwareMap.get(DcMotor.class, "rb");
//
//        leftFront = hardwareMap.get(DcMotor.class, "lf");
//        leftBack = hardwareMap.get(DcMotor.class, "lb");
//
//        intakeServo = hardwareMap.get(CRServo.class,"servo1");

        int idiot = 4;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();
    }

    
    @Override
    public void loop() {
        if (tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            telemetry.addData("detection count: ",tagProcessor.getDetections().size());
            if (tag.ftcPose != null){
                telemetry.addData("x",tag.ftcPose.x); //null object refrence for zome reazon
            }


        }

//        if (gamepad1.x) {
//            intakeServo.setPower(1.0);
//        }
        telemetry.addData("time",runtime.toString());
        telemetry.update();
    }
}