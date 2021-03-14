package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller that increments a scale using PID. This is different from a traditional PID
 * because the scale is incremented by a value determined using PID every cycle.
 */
public class VelocityPIDController {
    private final double p, i, d;

    double scale;
    boolean reset;

    private double lastError;
    private double errorSum;
    private double lastUpdateTime;

    private final double initialScaleFactor;
    private final double scaleMin;
    private final double scaleMax;

    /**
     * Constructs a VelocityPIDController with an initial scale factor of 1, a minimum scale of -1, and a
     * max of +1.
     *
     * @param p
     * @param i
     * @param d
     */
    public VelocityPIDController(double p, double i, double d) {
        this(p, i, d, 1);
    }

    /**
     * Constructs a VelocityPIDController with a scale minimum of -1 and a max of +1.
     *
     * @param p
     * @param i
     * @param d
     * @param initialScaleFactor The factor used to multiply by the first error provided after a
     *                           reset to determine the starting scale.
     */
    public VelocityPIDController(double p, double i, double d, double initialScaleFactor) {
        this(p, i, d, initialScaleFactor, -1, 1);
    }

    /**
     * Constructs a VelocityPIDController.
     *
     * @param p
     * @param i
     * @param d
     * @param defaultScale The factor used to multiply by the first error provided after a
     *      *                           reset to determine the starting scale.
     * @param scaleMin     The minimum possible value for the scale.
     * @param scaleMax     The maximum possible value for the scale.
     */
    public VelocityPIDController(double p, double i, double d, double initialScaleFactor, double scaleMin, double scaleMax) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.initialScaleFactor = initialScaleFactor;
        this.lastUpdateTime = SystemClock.elapsedRealtime();

        this.scaleMin = scaleMin;
        this.scaleMax = scaleMax;

        this.reset = true;
    }

    /**
     * Update the PID controller's scale. Should be called every iteration.
     *
     * @param error The error between the current state and the desired state
     * @return Updated PID scale
     */
    public double updateScale(double error) {
        double proport = 0;
        double deriv = 0;
        double integ = 0;

        long currentTime = SystemClock.elapsedRealtime();
        if (!reset) {
            proport = error * p;
            deriv = (error - lastError) / (lastUpdateTime - currentTime) * d;

            errorSum += error;
            integ = errorSum * i; // TODO integral scaled by time for consistency?
        } else {
//            scale = error * initialScaleFactor;

            reset = false;
        }

        double increment = proport + deriv + integ;
//        Log.d("CONTROLLER", "increment: " + increment);

        scale = Range.clip(scale + increment, scaleMin, scaleMax);

        lastUpdateTime = currentTime;
        lastError = error;

        return scale;
    }

    /**
     * Reset the PID controller using given default scale
     */
    public void reset(double velocity) {
        reset = true;
        scale = velocity * initialScaleFactor;
        errorSum = 0;
    }
}
