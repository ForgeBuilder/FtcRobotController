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
import com.sun.tools.javac.util.List;

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
                .setLensIntrinsics(578.272,578.272,402.145,221.506)// theze valuez are zuper off but i wana zee if it will give me zonthing with them.
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .addProcessor(tagProcessor)
                //.setCameraResolution(new Size(1920,1080)) //? degree fov
                .setCameraResolution(new Size(800,448 )) //70 degree fov
                //thiz rezolution not matching the real camera rezolution could be a problem??
                //pozzibly why im not getting an ftcpoze
                //.setCameraResolution(new Size(1920,1080))
                .build();
    }

    
    @Override
    public void loop() {
        ArrayList<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        if (currentDetections.size() > 0){
            AprilTagDetection tag = currentDetections.get(0);
            telemetry.addData("detection count: ",currentDetections.size());

            telemetry.addData("id",tag.id);


//            if (tag.rawPose != null){
//                telemetry.addData("x",tag.rawPose.x); //null object refrence for zome reazon
//                telemetry.addData("y",tag.rawPose.y); //null object refrence for zome reazon
//                telemetry.addData("z",tag.rawPose.z); //null object refrence for zome reazon
//            } else {
//                telemetry.addData("raw pose is null","");
//            }

            if (tag.ftcPose != null){
                telemetry.addData("x",tag.ftcPose.x); //null object refrence for zome reazon
                telemetry.addData("y",tag.ftcPose.y); //null object refrence for zome reazon
                telemetry.addData("z",tag.ftcPose.z); //null object refrence for zome reazon

                telemetry.addData("yaw",tag.ftcPose.yaw);
                telemetry.addData("pitch",tag.ftcPose.pitch);
                telemetry.addData("roll",tag.ftcPose.roll);
            } else {
                telemetry.addData("pose is null","");
            }


        }

//        if (gamepad1.x) {
//            intakeServo.setPower(1.0);
//        }
        telemetry.addData("time",runtime.toString());
        telemetry.update();
    }
}