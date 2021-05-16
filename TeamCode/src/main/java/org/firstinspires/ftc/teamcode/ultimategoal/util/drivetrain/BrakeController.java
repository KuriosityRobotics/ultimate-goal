package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

import android.util.Log;

/**
 * Calculates the desired power to brake using a VelocityPIDController
 */
public class BrakeController {
    VelocityPidController velocityPidController;
    TargetVelocityFunction targetVelocityFunction;

    private double velocityMax;

    private final double initialScaleFactor;
    private final double minVelocityCap;
    private final boolean cutOffAfterCoast;

    private boolean reset = true;
    private boolean atBrake = false;
    private boolean stopCoast = false;

    /**
     * Constructs a BrakeController.
     *
     * @param velocityPidController
     * @param targetVelocityFunction
     * @param initialScaleFactor     The factor multiplied against the target speed to determine the
     *                               starting scale after the PID is reset.
     * @param minVelocityCap         The minimum velocity that will be used as a speed cap for the
     *                               brake controller. This has an effect when the speed of the
     *                               robot as braking begins is very low.
     */
    public BrakeController(VelocityPidController velocityPidController, TargetVelocityFunction targetVelocityFunction, double initialScaleFactor, double minVelocityCap) {
        this(velocityPidController, targetVelocityFunction, initialScaleFactor, minVelocityCap, true);
    }

    /**
     * Constructs a BrakeController.
     *
     * @param velocityPidController
     * @param targetVelocityFunction
     * @param initialScaleFactor     The factor multiplied against the target speed to determine the
     *                               starting scale after the PID is reset.
     * @param minVelocityCap         The minimum velocity that will be used as a speed cap for the
     *                               brake controller. This has an effect when the speed of the
     *                               robot as braking begins is very low.
     * @param cutOffAfterCoast       Whether or not power is simply cut off if the desired velocity
     *                               is 0.
     */
    public BrakeController(VelocityPidController velocityPidController, TargetVelocityFunction targetVelocityFunction, double initialScaleFactor, double minVelocityCap, boolean cutOffAfterCoast) {
        this.velocityPidController = velocityPidController;
        this.targetVelocityFunction = targetVelocityFunction;
        this.initialScaleFactor = initialScaleFactor;
        this.minVelocityCap = minVelocityCap;
        this.cutOffAfterCoast = cutOffAfterCoast;
    }

    public double calculatePower(double distanceToTarget, double currentVelocity) {
        double absoluteDistanceToTarget = Math.abs(distanceToTarget);

        if (reset) {
            velocityMax = Math.max(minVelocityCap, currentVelocity);

            double newScale = Math.min(targetVelocityFunction.desiredVelocity(distanceToTarget), velocityMax) * initialScaleFactor;
            // TODO doesn't work for neg ^^
            velocityPidController.reset(newScale);
            reset = false;
        }

        // Got to brake point?
        boolean stoppedNearBrakePoint = absoluteDistanceToTarget < targetVelocityFunction.stopThreshold
                && Math.abs(currentVelocity) < 0.08;
        if (!atBrake && stopCoast && Math.abs(currentVelocity) < 0.05) {
            atBrake = true;
            velocityPidController.reset(0);
        } else if (absoluteDistanceToTarget > targetVelocityFunction.coastThreshold) { // Too far from brake point to be considered @ it?
            atBrake = false;
            stopCoast = false;
        }

        Log.v("brakecontroller", "atbrake: " + atBrake);
        Log.v("brakecontroller", "stopcoast: " + stopCoast);

        if ((absoluteDistanceToTarget < targetVelocityFunction.stopThreshold && !atBrake) || (stopCoast && !atBrake)) { // close enough
            stopCoast = true;
            Log.v("brakecontroller", "STAHP");
            return 0;
        } else {
            // on the way to brake point
            if (atBrake && ((absoluteDistanceToTarget < targetVelocityFunction.stopThreshold && currentVelocity < 0.5)
                    || (absoluteDistanceToTarget < targetVelocityFunction.stopThreshold / 2))) {
                return 0;
            }

            double targetVelocity = Math.min(
                    atBrake ? targetVelocityFunction.atBrakeDesiredVelocity(distanceToTarget)
                            : targetVelocityFunction.desiredVelocity(distanceToTarget),
                    velocityMax); // TODO doesn't work for negative velocities

            double error = targetVelocity - currentVelocity;

            double scale = velocityPidController.calculateScale(error);

            Log.v("brakecontroller", "scale: " + scale);

            return scale;
        }
    }

    public double targetVelocity(double distanceToTarget) {
        double absoluteDistanceToTarget = Math.abs(distanceToTarget);
        if ((absoluteDistanceToTarget < targetVelocityFunction.stopThreshold && atBrake) || (stopCoast && !atBrake)) { // close enough
            return 0;
        } else {
            return Math.min(
                    atBrake ? targetVelocityFunction.atBrakeDesiredVelocity(distanceToTarget)
                            : targetVelocityFunction.desiredVelocity(distanceToTarget),
                    velocityMax);
        }
    }

    /**
     * Reset the controller, useful for when a new brakepoint is set. This resets the PID and sets a
     * new speed cap.
     */
    public void reset() {
        reset = true;
        atBrake = false;
        stopCoast = false;
    }

    public void setScale(double scale) {
        velocityPidController.reset(scale);
    }

    public boolean getAtBrake() {
        return atBrake;
    }

    public boolean getStopCoast() {
        return stopCoast;
    }
}
