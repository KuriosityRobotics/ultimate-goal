package org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * A PID controller that increments a scale using PID. This is different from a traditional PID
 * because the scale is incremented by a value determined using PID every cycle.
 */
public class VelocityPidController {
    private final double p, i, d;

    double scale;
    boolean reset;

    private double lastError;
    private double errorSum;
    private double lastUpdateTime;

    private final double defaultScale;
    private final double scaleMin;
    private final double scaleMax;

    /**
     * Constructs a VelocityPIDController with a default scale of 1, a minimum scale of -1, and a
     * max of +1.
     *
     * @param p
     * @param i
     * @param d
     */
    public VelocityPidController(double p, double i, double d) {
        this(p, i, d, 1);
    }

    /**
     * Constructs a VelocityPIDController with a scale minimum of -1 and a max of +1.
     *
     * @param p
     * @param i
     * @param d
     * @param defaultScale The scale to start at and reset to.
     */
    public VelocityPidController(double p, double i, double d, double defaultScale) {
        this(p, i, d, defaultScale, -1, 1);
    }

    /**
     * Constructs a VelocityPIDController.
     *
     * @param p
     * @param i
     * @param d
     * @param defaultScale The scale to start at and reset to.
     * @param scaleMin     The minimum possible value for the scale.
     * @param scaleMax     The maximum possible value for the scale.
     */
    public VelocityPidController(double p, double i, double d, double defaultScale, double scaleMin, double scaleMax) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.scale = defaultScale;
        this.defaultScale = defaultScale;
        this.lastUpdateTime = SystemClock.elapsedRealtime();

        this.scaleMin = scaleMin;
        this.scaleMax = scaleMax;
    }

    /**
     * Update the PID controller's scale. Should be called every iteration.
     *
     * @param error The error between the current state and the desired state
     * @return Updated PID scale
     */
    public double updateScale(double error) {
        double proport = error * p;

        long currentTime = SystemClock.elapsedRealtime();
        double deriv = 0;
        double integ = 0;
        if (!reset) {
            deriv = ((error - lastError) / (currentTime - lastUpdateTime)) * d;

            errorSum += error;
            integ = errorSum * i; // TODO integral scaled by time for consistency?
        }

        Log.v("BRAKING", "P: " + proport + ", D: " + deriv);

        double increment = proport + deriv + integ;

        scale = Range.clip(increment, scaleMin, scaleMax);

        lastUpdateTime = currentTime;
        lastError = error;

        return scale;
    }

    /**
     * Reset the PID controller using given default scale
     */
    public void reset() {
        reset = true;
        scale = defaultScale;
        errorSum = 0;
    }
}
