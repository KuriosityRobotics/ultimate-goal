package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

public class TargetVelocityFunction {
    private final double slowRate, coastThreshold, coastVelocity, stopThreshold;

    public TargetVelocityFunction(double slowRate, double coastThreshold, double coastVelocity, double stopThreshold) {
        this.slowRate = slowRate;
        this.coastThreshold = coastThreshold;
        this.coastVelocity = coastVelocity;
        this.stopThreshold = stopThreshold;
    }

    public double desiredVelocity(double distanceToTarget) {
        if (distanceToTarget < stopThreshold) {
            return 0;
        } else if (distanceToTarget < coastThreshold) {
            return coastVelocity;
        } else {
            // linear function with transformations
            return slowRate * (distanceToTarget - coastThreshold) + coastVelocity;
        }
    }
}
