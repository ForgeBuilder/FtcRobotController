package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


// Inside your OpMode


@Configurable
public class CrossbowTeleop extends CrossbowMain {
    private ElapsedTime runtime = new ElapsedTime();

    public static double[] start_pose_members = {0,0,0};

    @Override
    public void start() {
        super.start();
        runtime.reset();
    }

    @Override
    public void init() {
        super.init();
        try {
            if (team == "blue"){
                autopose = CrossbowAutoBlue.auto_current_pose;
            } else if (team == "red"){
                autopose = CrossbowAutoRed.auto_current_pose;
            }

        } finally {
            if (autopose != null){
                telemetry.addData("auto_pose",autopose);
            } else {
                telemetry.addData("auto_pose","no auto pose found");
            }
        }
        Pose start_pose = new Pose(start_pose_members[0],start_pose_members[1],start_pose_members[2]);
        if (start_pose != new Pose(0,0,0)){
            follower.setPose(start_pose);
        } else if (autopose!= null){
            follower.setPose(autopose);
        }
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }


    private GamepadManager c_gamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
    private GamepadManager c_gamepad2 = PanelsGamepad.INSTANCE.getFirstManager();

    final double METERS_TO_INCHES = 39.3701;

    boolean fire_launcher = false;
    public static boolean debug_fire = false;

    private Pose autopose;

    public static double chasis_turret_velocity_cancelation_constant = 1;

    @Override
    public void loop() {
        super.loop();
        //handels limelight, should probably go in main at some point
        limelight_code();

        if (gamepad1.aWasPressed()){
            fire_launcher = !fire_launcher;
        }



        //handles saving position and making return path to saved position
        //teleop_return_to_position();

        //drivetrain stuff
        if (follower.isBusy()) {
            if (gamepad1.x) {
                follower.breakFollowing();
            }
        } else {
            //this is a little nonsensical. I might as well have just put all the teleop functions in here
            //and made the motors public. It is what it is.. this is how we learn!

            //For the teleop functions I could just have them in here and give them refrences to what they need.
            manual_drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.left_trigger
            );
        }
        //must go after drivetrain

        //launcher and turret stuff

        ///turret stuff

        //negitive is to the right and positive is to the left for the motor encoder

        if (gamepad1.left_bumper) {
            turret.set_turret_state(TurretState.FINDING_LIMIT);
            turret.track_goal_from_current_position();
        } else if (gamepad1.b){
            //for centering when the started pose is wrong - doesn't override heading, need to implement
            turret.find_pose_with_ll();
        } else {
            turret.track_goal_from_current_position();
        }


        //launcher

        fire_launcher = gamepad1.y || (gamepad2.right_trigger > 0.1) || (gamepad1.right_bumper) || debug_fire;

        launcher_code(fire_launcher, gamepad1.y);

        if (gamepad1.dpadUpWasPressed()) {
            set_launcher_speed(get_launcher_speed() + 40);
        } else if (gamepad1.dpadDownWasPressed()) {//||gamepad1.dpadDownWasPressed()
            set_launcher_speed(get_launcher_speed() - 40);
        }
        if (LLresult != null && LLresult.isValid()) {
            Pose3D botpose = LLresult.getBotpose_MT2();

            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double yaw = botpose.getOrientation().getYaw();
                double meters_to_inches = 39.3701;
                telemetry.addData("MT2 Location\n", "x: " + x * meters_to_inches + "\ny: " + y * meters_to_inches + "\nyaw: " + yaw);
            }
        }
//        if (gamepad1.yWasPressed()) {
//            limelight_set_pose();
//        }

        // Show the elapsed game time and update telemetry so we can see it
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        Pose current_pose = follower.getPose();
        telemetry.addData("Follower Pose", current_pose.getX() + ", " + current_pose.getY());

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }


    private ElapsedTime intake_reverse_timer = new ElapsedTime();

    @Override
    public void intake_code() {
        panelsTelemetry.addData("spin intake",spin_intake);
        panelsTelemetry.addData("reverse intake",reverse_intake);

        boolean activate = (gamepad1.right_trigger > 0.1) || gamepad2.a;

        if (activate){
            spin_intake = true;
        } else {
            spin_intake = false;
        }

//        if (!spin_intake && activate) {
//            if (intake_reverse_timer.seconds()>1){
//                reverse_intake = true;
//            }
//        } else {
//            reverse_intake = false;
//            intake_reverse_timer.reset();
//        }
        super.intake_code();
    }
}
