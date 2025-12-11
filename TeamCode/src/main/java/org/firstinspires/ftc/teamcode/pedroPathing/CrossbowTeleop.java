package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import javax.lang.model.element.VariableElement;


// Inside your OpMode

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



    private GamepadManager c_gamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
    private GamepadManager c_gamepad2 = PanelsGamepad.INSTANCE.getFirstManager();

    @Override
    public void loop(){
        super.loop();

        //this stuff doesn't really work. I wish lazlar was smart n gave his gamepad object the same functions.
        //I could make a translator!
        c_gamepad1.asCombinedFTCGamepad(gamepad1);
        c_gamepad2.asCombinedFTCGamepad(gamepad1);

        panelsTelemetry.debug("==== Buttons ====");
        panelsTelemetry.debug("A: "+c_gamepad1.getCross());
        panelsTelemetry.debug("B: "+c_gamepad1.getCircle());
        panelsTelemetry.debug("X: "+c_gamepad1.getSquare());
        panelsTelemetry.debug("Y: "+c_gamepad1.getTriangle());
        panelsTelemetry.debug("DPad Up: ${g1.dpad_up}");
        panelsTelemetry.debug("DPad Down: ${g1.dpad_down}") ;
        panelsTelemetry.debug("DPad Left: ${g1.dpad_left}");
        panelsTelemetry.debug("DPad Right: ${g1.dpad_right}");
        panelsTelemetry.debug("Left Bumper: ${g1.left_bumper}");
        panelsTelemetry.debug("Right Bumper: ${g1.right_bumper}");
        panelsTelemetry.debug("Left Trigger: ${g1.left_trigger}");
        panelsTelemetry.debug("Right Trigger: ${g1.right_trigger}");
        panelsTelemetry.debug("Start / Options: ${g1.options}");
        panelsTelemetry.debug("Back / Share: ${g1.back}");
        panelsTelemetry.debug("Guide / PS: ${g1.guide}");
        panelsTelemetry.debug("Touchpad: ${g1.touchpad}");
        panelsTelemetry.debug("Left Stick Button: ${g1.left_stick_button}");
        panelsTelemetry.debug("Right Stick Button: ${g1.right_stick_button}");
        panelsTelemetry.debug("==== Sticks ====");
        panelsTelemetry.debug("Left Stick X: ${g1.left_stick_x}");
        panelsTelemetry.debug("Left Stick Y: ${g1.left_stick_y}");
        panelsTelemetry.debug("Right Stick X: ${g1.right_stick_x}");
        panelsTelemetry.debug("Right Stick Y: ${g1.right_stick_y}");


        limelight_code();
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
                telemetry.addData("MT2 Location", "(" + x*meters_to_inches + ", " + y*meters_to_inches + ")");
            }
        }
//        if (gamepad1.yWasPressed()){
//            limelight_set_pose();
//        }

        // Show the elapsed game time and update telemetry so we can see it
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        Pose current_pose = follower.getPose();
        telemetry.addData("Follower Pose",current_pose.getX() + ", "+current_pose.getY());

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    private boolean spin_intake = false;

    private ElapsedTime intake_reverse_timer = new ElapsedTime();
    @Override
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
