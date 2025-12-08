/*
Copyright 2025 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp()

public class TurretTest extends OpMode {
    /* Declare OpMode members. */


    private CRServo liftServo;
    private DcMotorEx MOTOR;

    private PIDFCoefficients launcherCoefficients = new PIDFCoefficients(290,3,0,0);

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        liftServo = hardwareMap.get(CRServo.class,"1"); //peter, you're just like your brother lol
        MOTOR = hardwareMap.get(DcMotorEx.class,"MOTOR");
        MOTOR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    int target_velocity = 600; //2800 is the max

    boolean spin_flywheel = false;

    @Override
    public void loop() {


        liftServo.setPower(gamepad1.right_stick_y);

        if (gamepad1.aWasPressed()){
            spin_flywheel = !spin_flywheel;
        }

        if (gamepad1.dpadUpWasPressed()){
            target_velocity += 100;
        }
        if (gamepad1.dpadDownWasPressed()){
            target_velocity -= 100;
        }

        if (spin_flywheel){
            MOTOR.setPower(1);
            MOTOR.setVelocity(target_velocity);
        } else {
            MOTOR.setPower(0);
        } 


        telemetry.addData("Spin Flywheel: ",spin_flywheel);
        telemetry.addData("Target Velocity: ",target_velocity);
        telemetry.addData("Current Velocity: ",MOTOR.getVelocity());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
