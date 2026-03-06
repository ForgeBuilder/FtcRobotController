package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class BallistaAuto extends CrossbowMain {

    protected boolean fire_artifact;

    public enum turret_firing_state {
        FIRING_ARTIFACTS,FINISHED_FIRING
    }

    public void start_turret_fire(Pose pose){

    }

    public turret_firing_state turret_fire_loop(){
        if (open_door && time_since_ball_ready.seconds() > 1.0){
            turret_stop_firing();
            return turret_firing_state.FINISHED_FIRING;
        } else {
            return turret_firing_state.FIRING_ARTIFACTS;
        }
    }
    public void turret_stop_firing(){
        follower.breakFollowing();
        fire_artifact = false;
    }
}