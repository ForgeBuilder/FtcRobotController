package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class CrossbowAutoFar extends CrossbowAuto{
    @Override public void start(){
        super.start();
        follower.setPose(new Pose(101,-5.5*apm, apm*(Math.PI/2)));
    }
}
