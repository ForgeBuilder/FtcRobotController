package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double Kp, Ki, Kd;
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double update(double target, double current) {
        // Calculate error
        double error = target - current;

        // Rate of change (Derivative)
        double derivative = (error - lastError) / timer.seconds();

        // Total error over time (Integral)
        integralSum += (error * timer.seconds());

        // Reset timer for next loop
        timer.reset();
        lastError = error;

        // PID output formula
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}