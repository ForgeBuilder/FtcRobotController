package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "fullautonomous (Blocks to Java)")
public class fullautonomous extends LinearOpMode {

  private DcMotor leftBack;
  private DcMotor leftFront;
  private DcMotor rightBack;
  private DcMotor rightFront;
  private Servo dropServo;
  private DistanceSensor rightDist;

  int leftPos;
  int rightPos;

  /**
   * Describe this function...
   */
  private void drive(int leftTarget, int rightTarget, double speed) {
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftPos += leftTarget;
    rightPos += rightTarget;
    leftBack.setTargetPosition(leftPos);
    leftFront.setTargetPosition(leftPos);
    rightBack.setTargetPosition(rightPos);
    rightFront.setTargetPosition(rightPos);
    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightBack.setPower(0.5);
    rightFront.setPower(0.5);
    rightBack.setPower(0.5);
    rightFront.setPower(0.5);
    ((DcMotorEx) leftBack).setVelocity(400);
    ((DcMotorEx) leftFront).setVelocity(400);
    ((DcMotorEx) rightBack).setVelocity(400);
    ((DcMotorEx) rightFront).setVelocity(400);
    while (leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy() && opModeIsActive()) {
      waitForStart();
      idle();
    }
  }

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    dropServo = hardwareMap.get(Servo.class, "dropServo");
    rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");

    waitForStart();
    // Put initialization blocks here.
    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    leftPos = 0;
    rightPos = 0;
    waitForStart();
    //dropServo.setPosition();
  }
}
