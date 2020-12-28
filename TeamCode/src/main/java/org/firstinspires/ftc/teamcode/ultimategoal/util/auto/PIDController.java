package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class PIDController implements TelemetryProvider {
    public double P;
    public double I;
    public double D;

    double integral;
    double prevError;
    public double scale = 1;

    public PIDController(double P, double I, double D, Robot robot) {
        robot.telemetryDump.registerProvider(this);
        this.P = P;
        this.I = I;
        this.D = D;

        reset();
    }

    public void reset() {
        integral = 0;
        prevError = 1000000;
    }

    public double calculatePID(double distance, double targetDistance) {
        return calculatePID(targetDistance - distance);
    }

    public double calculatePID(Point current, Point target) {
        return calculatePID(Math.hypot(current.x - target.x, current.y - target.y));
    }

    public double calculatePID(double error) {
        integral += error * 0.004;
        double derivative = (error - prevError) / 0.004;

        scale = P * error + I * integral + D * derivative;

        return scale;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Scale: " + this.scale);
        data.add("Integral: " + this.integral);
        data.add("prev error: " + prevError);
        return data;
    }

    public String getName() {
        return "PIDController";
    }
}
