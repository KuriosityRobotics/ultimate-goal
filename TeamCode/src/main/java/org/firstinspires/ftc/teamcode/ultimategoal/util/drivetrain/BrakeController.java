package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

/**
 * Calculates the desired power to brake using a VelocityPIDController
 */
public class BrakeController {
    VelocityPidController velocityPIDController;
    TargetVelocityFunction targetVelocityFunction;

    private double velocityMax;

    private final double initialScaleFactor;
    private final double minVelocityCap;
    private boolean reset = true;
    private final boolean cutOffAfterCoast;

    /**
     * Constructs a BrakeController.
     *
     * @param velocityPIDController
     * @param targetVelocityFunction
     * @param initialScaleFactor     The factor multiplied against the target speed to determine the
     *                               starting scale after the PID is reset.
     * @param minVelocityCap         The minimum velocity that will be used as a speed cap for the
     *                               brake controller. This has an effect when the speed of the
     *                               robot as braking begins is very low.
     */
    public BrakeController(VelocityPidController velocityPIDController, TargetVelocityFunction targetVelocityFunction, double initialScaleFactor, double minVelocityCap) {
        this(velocityPIDController, targetVelocityFunction, initialScaleFactor, minVelocityCap, true);
    }

    /**
     * Constructs a BrakeController.
     *
     * @param velocityPIDController
     * @param targetVelocityFunction
     * @param initialScaleFactor     The factor multiplied against the target speed to determine the
     *                               starting scale after the PID is reset.
     * @param minVelocityCap         The minimum velocity that will be used as a speed cap for the
     *                               brake controller. This has an effect when the speed of the
     *                               robot as braking begins is very low.
     * @param cutOffAfterCoast       Whether or not power is simply cut off if the desired velocity
     *                               is 0.
     */
    public BrakeController(VelocityPidController velocityPIDController, TargetVelocityFunction targetVelocityFunction, double initialScaleFactor, double minVelocityCap, boolean cutOffAfterCoast) {
        this.velocityPIDController = velocityPIDController;
        this.targetVelocityFunction = targetVelocityFunction;
        this.initialScaleFactor = initialScaleFactor;
        this.minVelocityCap = minVelocityCap;
        this.cutOffAfterCoast = cutOffAfterCoast;
    }

    public double calculatePower(double distanceToTarget, double currentVelocity) {
        double targetVelocity = targetVelocityFunction.desiredVelocity(distanceToTarget);

        if (reset) {
            velocityMax = Math.max(minVelocityCap, currentVelocity);
            velocityPIDController.reset(targetVelocity * initialScaleFactor);

            reset = false;
        }

        targetVelocity = Math.min(targetVelocity, velocityMax);

        if (targetVelocity == 0 && cutOffAfterCoast) {
            return 0; // The TargetVelocityFunction has a 'coast' period, designed so that the robot simply cuts power after reaching its target.
        } else {
            double error = targetVelocity - currentVelocity;

            return velocityPIDController.calculateScale(error);
        }
    }

    public double targetVelocity(double distanceToTarget) {
        return targetVelocityFunction.desiredVelocity(distanceToTarget);
    }

    /**
     * Reset the controller, useful for when a new brakepoint is set. This resets the PID and sets a
     * new speed cap.
     */
    public void reset() {
        reset = true;
    }
}
