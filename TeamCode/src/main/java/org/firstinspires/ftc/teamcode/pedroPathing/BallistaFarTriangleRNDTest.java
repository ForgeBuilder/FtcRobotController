package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaFarTriangleRNDTest")
public class BallistaFarTriangleRNDTest extends CrossbowMain {

    boolean fire_artifact;
    PathChain test_path;

    Pose starter_pose = new Pose(68.5,7.318,Math.PI);

    Pose test_pose = new Pose(-42,-55,Math.PI);
//    Pose launch_pose = new Pose(-14.3,-16,Math.PI);

//    Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
        set_team("red");
        follower.setPose(starter_pose);
    }


    public AutoStep current_auto_step = AutoStep.FireFirstVolley;
    enum AutoStep {
        FireFirstVolley,ZoneOneIntake
    }

    @Override public void start(){
        super.start();

        //for red, mod constant is still 1 as long as you spesify 3.14 as the heading shift within the config settings.

        follower.holdPoint(starter_pose);
        spin_launcher = true;
        fire_artifact = true;

//        follower.setStartingPose(starter_pose);
//        PathChain to_first_launch = follower.pathBuilder()
//                .addPath(new BezierLine(starter_pose,test_pose))
//                .setConstantHeadingInterpolation(Math.PI)
//                .build();
//        follower.followPath(to_first_launch);
    }

    @Override public void loop(){
        super.loop();
        turret.update();
        launcher_code(fire_artifact,false);

        panelsTelemetry.addData("step: ",current_auto_step);
        telemetry.addData("step: ",current_auto_step);

        switch (current_auto_step){
            case FireFirstVolley:
                if (open_door && (time_since_ball_ready.seconds() > 1.0)){
                    follower.breakFollowing();
                    fire_artifact = false;
                    ZoneOneIntake();
                }
                break;
            case ZoneOneIntake:

                break;
        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }

    public void ZoneOneIntake(){

        Pose ready_to_intake = new Pose(62,22,Math.PI);

        PathChain zone_one_intake_path = follower.pathBuilder()
            .addPath(new BezierLine(follower.getPose(),ready_to_intake))
            .setConstantHeadingInterpolation(Math.PI)
            .addParametricCallback(0.5, () ->{
                spin_intake = true;
            })
            .build();
        follower.followPath(zone_one_intake_path);
        current_auto_step = AutoStep.ZoneOneIntake;
    }

}
