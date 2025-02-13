package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;

public class PIDController {

    double Kp;
    double Ki;
    double Kd;
    double lastError = 0;
    double integralSum = 0;
    boolean angleWrap = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    ElapsedTime timer = new ElapsedTime();

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

    }
    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    /**
     * update the PID controller output
     */
    public double output(double reference, double state) {
        double error;
        double derivative;
        // check if we need to unwrap angle
        if (angleWrap) {
            error = angleWrap(reference - state);
        } else {
            error = reference - state;
        }
        // forward euler integration
        integralSum += error * timer.seconds();
        derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd);

        timer.reset();
        lastError = error;

        packet.put("reference:", reference);
        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}