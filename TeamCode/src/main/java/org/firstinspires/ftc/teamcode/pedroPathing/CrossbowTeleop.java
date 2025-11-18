package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CrossbowTeleop")

public class CrossbowTeleop extends CrossbowMain {
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void start(){
        super.start();
        runtime.reset();
    }

    @Override
    public void loop(){
        teleop_limelight_code();
        launcher_code((gamepad2.right_trigger>0.1)||(gamepad1.right_trigger>0.1));
        intake_code();
        //handles saving position and making return path to saved position
        teleop_return_to_position();

        //drivetrain stuff
        if (follower.isBusy()) {
            follower_code(gamepad1.x);
        } else {
            drive_with_teleop();
        }

        // Show the elapsed game time and update telemetry so we can see it
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
