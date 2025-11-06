import static
com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")

public class TeleOpJava extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;
    double leftPower;
    double rightPower;
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        telemetry.addData("Status", "Initialized");
    }

    public void loop(){
        telemetry.addData("nothing - ","nothing");
    }
}
