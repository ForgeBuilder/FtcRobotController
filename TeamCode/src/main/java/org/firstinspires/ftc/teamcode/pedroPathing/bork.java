package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name ="borked")
public class bork extends OpMode {
    private DcMotor MOTOR = null;
    ElapsedTime time = new ElapsedTime();

    @Override public void init() {
        MOTOR = hardwareMap.get(DcMotor.class, "MOTOR");
        MOTOR.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("I just jorked it all over aaaaaaaaaaaa", "oh yeah");
    }
    @Override public void loop() {
        MOTOR.setPower(1);
//        telemetry.addData( "Time",) (this gonna be fixed later mane)
    }
}
