package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class CrossbowAutoFar extends CrossbowAuto{
    @Override public void start(){
        super.start();
        follower.setPose(new Pose(0,-54*apm, Math.toRadians(0)));
    }
}
