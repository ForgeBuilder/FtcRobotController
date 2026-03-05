package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BallistaTurretAutoTest")
public class BallistaTurretAutoTest extends CrossbowMain {

    Pose pose_a = new Pose(0,0,0);

    Pose pose_b = new Pose(10,0,Math.PI/4);

    Pose pose_c = new Pose(10,10,Math.PI/4);

    Pose pose_d = new Pose(0,10,0);

    PathChain test_path;

    @Override public void start(){
        super.start();


    }


    @Override public void loop(){
        super.loop();
        panelsTelemetry.update(telemetry);
    }

}
