package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


// Inside your OpMode

@TeleOp(name="CrossbowTeleopBlue")

public class CrossbowTeleop extends CrossbowMain {
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void start(){
        super.start();
        runtime.reset();
    }

    @Override public void init(){
        super.init();
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void loop(){
        super.loop();
        teleop_limelight_code();
        launcher_code((gamepad2.right_trigger>0.1)||(gamepad1.right_trigger>0.1),gamepad1.right_bumper);
        intake_code();
        //handles saving position and making return path to saved position
        teleop_return_to_position();

        //drivetrain stuff
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

        if (gamepad1.dpadUpWasPressed()){
            set_launcher_speed(get_launcher_speed()+40);
        } else if (gamepad1.dpadDownWasPressed()) {//||gamepad1.dpadDownWasPressed()
            set_launcher_speed(get_launcher_speed()-40);
        }

        LLresult = limelight.getLatestResult();
        if (LLresult != null && LLresult.isValid()) {
            Pose3D botpose = LLresult.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double meters_to_inches = 39.3701;
                telemetry.addData("MT1 Location", "(" + x*meters_to_inches + ", " + y*meters_to_inches + ")");
            }
        }
        if (gamepad1.yWasPressed()){
            limelight_set_pose();
        }

        // Show the elapsed game time and update telemetry so we can see it
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        Pose current_pose = follower.getPose();
        telemetry.addData("Follower Pose",current_pose.getX() + ", "+current_pose.getY());

        telemetry.update();
    }

    private boolean spin_intake = false;

    private ElapsedTime intake_reverse_timer = new ElapsedTime();
    @Override
    public void intake_code(){
        if (gamepad2.aWasPressed()||gamepad1.leftBumperWasPressed()){
            spin_intake = !spin_intake;
        }
        //it's a 312 so 537.7 PPR at the Output Shaft. 5.2 RPS (max) would be 2796.04 or about 2800.
        if (spin_intake&&(!kick)){
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
