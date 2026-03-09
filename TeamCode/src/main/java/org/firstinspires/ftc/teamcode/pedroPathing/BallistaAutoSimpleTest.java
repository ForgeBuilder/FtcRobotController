package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallistaAutoSimpleTest extends BallistaAuto {
    PathChain test_path;
    Pose starter_pose;

    ElapsedTime step_timer = new ElapsedTime();

    Pose launch_2_pose;

    Pose human_zone_corner_pose;
//    Pose launch_pose = new Pose(-14.3,-16,Math.PI);

//    Pose firing_pose = new Pose();

    @Override public void init(){
        super.init();
    }


    public AutoStep current_auto_step = AutoStep.None;
    enum AutoStep {
        None, FireFirstVolley, HumanZoneIntakeOne, ReturnToFarLaunchOne,FireSecondVolley,IntakeZoneOne
    }

    @Override public void start(){
        set_launcher_speed(1480);
        super.start();

        starter_pose = new Pose(68.2,-7.1*apm,apm*-Math.PI/2);
        launch_2_pose = new Pose(65.5,-10*apm,apm*-Math.PI/2);

        //CHANGE IT BACK TO 60 OR AS HIGH AS WE NEED TO GET IT TO HIT THE WALL BUT COME BACK
        human_zone_corner_pose = new Pose(66,-61*apm,apm*-Math.PI/2);

        follower.setPose(starter_pose);

        set_step(AutoStep.FireFirstVolley);

    }

    @Override public void loop(){
        super.loop();
        turret.update();
        launcher_code(fire_artifact,false);

        panelsTelemetry.addData("step: ",current_auto_step);
        telemetry.addData("step: ",current_auto_step);

        if (runtime.seconds() < 28){
            set_step(AutoStep.None);
            turret_stop_firing();
            follower.breakFollowing();
            follower.pathBuilder()
                    .addPath(new BezierLine(current_pedro_pose.getPose(),human_zone_corner_pose))
                    .setConstantHeadingInterpolation(human_zone_corner_pose.getHeading())
                    .build();

        }

        turret.track_goal_from_current_position();
        panelsTelemetry.update(telemetry);
    }
    public void set_step(AutoStep step) {

        switch (step) {
            case FireFirstVolley:
                spin_launcher = true;
                fire_artifact = true;

                Runnable FireFirstVolleyLoop = ()->{
                    turret_stop_firing();
                    fire_artifact = false;
                    follower.resumePathFollowing();
                };

                PathBuilder.CallbackCondition StopFireFirstVolleyLoop = new PathBuilder.CallbackCondition() {
                    @Override
                    public boolean isReady() {
                        return (open_door && time_since_ball_ready.seconds() > 1.0);
                    }
                };

                PathChain FireFirstVolley = follower.pathBuilder()
                        .addPath(new BezierPoint(starter_pose))
                        .addCallback(StopFireFirstVolleyLoop,FireFirstVolleyLoop)
                        .build();
                follower.followPath(FireFirstVolley);
                follower.pausePathFollowing();
                time_since_ball_ready.reset();
                break;
            case HumanZoneIntakeOne:
                follower.setMaxPower(1);

                follower.breakFollowing();
                PathChain human_zone_intake_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),human_zone_corner_pose))
                        .setConstantHeadingInterpolation(human_zone_corner_pose.getHeading())
                        .addParametricCallback(0.4, () ->{
                            spin_intake = true;
                        })
                        .build();
                follower.followPath(human_zone_intake_path);
                step_timer.reset();
                break;
            case ReturnToFarLaunchOne:
                follower.setMaxPower(1);
                PathChain return_to_launch_path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),launch_2_pose))
                        .setConstantHeadingInterpolation(launch_2_pose.getHeading())
                        .addParametricCallback(0.2, () ->{
                            spin_intake = false;
                        })
                        .addParametricCallback(0.4, () ->{
                            follower.setMaxPower(0.2);
                        })
                        .addParametricCallback(0.6, () ->{
                            spin_intake = true;
                        })
                        .addParametricCallback(0.9, () ->{
                            spin_intake = false;
                        })
                        .build();
                follower.followPath(return_to_launch_path);
                break;
            case FireSecondVolley:
                follower.setMaxPower(1);
                time_since_ball_ready.reset();
                spin_intake = false;
                follower.holdPoint(launch_2_pose);
                spin_launcher = true;
                fire_artifact = true;
                break;
        }
        current_auto_step = step;
    }
}
