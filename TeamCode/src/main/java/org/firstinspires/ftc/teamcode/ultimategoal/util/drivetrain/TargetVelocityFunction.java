package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

public class TargetVelocityFunction {
    final double slowRate, coastThreshold, coastVelocity, stopThreshold;

    public TargetVelocityFunction(double slowRate, double coastThreshold, double coastVelocity, double stopThreshold) {
        this.slowRate = slowRate;
        this.coastThreshold = coastThreshold;
        this.coastVelocity = coastVelocity;
        this.stopThreshold = stopThreshold;
    }

    public double desiredVelocity(double distanceToTarget) {
        if (distanceToTarget == 0) {
            return 0;
        }

        double trueDistance = Math.abs(distanceToTarget);
        double sign = Math.abs(distanceToTarget) / distanceToTarget;

        double targetTrueVelocity;

        if (trueDistance < stopThreshold) {
            targetTrueVelocity = 0;
        } else if (trueDistance < coastThreshold) {
            targetTrueVelocity = coastVelocity;
        } else {
            // linear function with transformations
            targetTrueVelocity = slowRate * (trueDistance - coastThreshold) + coastVelocity;
        }

        return targetTrueVelocity * sign;
    }

    public double noCoastDesiredVelocity(double distanceToTarget) {
        return distanceToTarget * slowRate;
    }
}
