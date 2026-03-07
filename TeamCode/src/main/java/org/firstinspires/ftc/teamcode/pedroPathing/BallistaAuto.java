package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class BallistaAuto extends BallistaMain {

    protected boolean fire_artifact;

    public enum turret_firing_state {
        FIRING_ARTIFACTS,FINISHED_FIRING
    }

    public enum balls_loaded_state{
        NOT_FULL,FULL
    }

    public balls_loaded_state get_balls_loaded_state(){
        if (ball_ready && ball_in_intake){
            return balls_loaded_state.FULL;
        } else {
            return balls_loaded_state.NOT_FULL;
        }
    }

    public balls_loaded_state current_balls_loaded_state = balls_loaded_state.NOT_FULL;
    public void start_turret_fire(Pose pose){
        time_since_ball_ready.reset();
        follower.holdPoint(pose);
        spin_launcher = true;
        fire_artifact = true;
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