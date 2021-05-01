package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.LinearInterpolation;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.ITarget;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;

public class Shooter extends ModuleCollection implements TelemetryProvider {
    private final Robot robot;
    private final boolean isOn;

    private final ShooterModule shooterModule;
    private final HopperModule hopperModule;

    // States
    public ITarget target = BLUE_HIGH;

    public boolean lockTarget = true;
    public boolean flywheelOn = false;
    public int queuedIndexes = 0;

    public boolean manualTurret = false;
    public double manualTurretPower = 0;

    public double manualAngleCorrection;
    public double manualAngleFlapCorrection;

    // helpers
    private boolean queueDelivery = false;
    private long lastIndexTime = 0;

    // Constants
    public final static int HIGHGOAL_FLYWHEEL_SPEED = 1750;
    public final static int POWERSHOT_FLYWHEEL_SPEED = 1650; // todo

    private final static long RING_FIRE_TIME = 400;

    private static final double TURRET_DISTANCE_FROM_BACK = 7;

    private static final LinearInterpolation HIGH_GOAL_DATA = new LinearInterpolation(
            new double[][]{
                    {85.0 - TURRET_DISTANCE_FROM_BACK, 0.2384, Math.toRadians(7.312)},
                    {90.0 - TURRET_DISTANCE_FROM_BACK, 0.2401, Math.toRadians(9.2)},
                    {95.0 - TURRET_DISTANCE_FROM_BACK, 0.2411, Math.toRadians(9.19)},
                    {100.0 - TURRET_DISTANCE_FROM_BACK, 0.24, Math.toRadians(9.2)},
                    {105.0 - TURRET_DISTANCE_FROM_BACK, 0.24, Math.toRadians(8.1)},
                    {110.0 - TURRET_DISTANCE_FROM_BACK, 0.2374, Math.toRadians(8.881)},
                    {115.0 - TURRET_DISTANCE_FROM_BACK, 0.2374, Math.toRadians(7.191)},
                    {120.0 - TURRET_DISTANCE_FROM_BACK, 0.2362, Math.toRadians(7.181)},
                    {125.0 - TURRET_DISTANCE_FROM_BACK, 0.2352, Math.toRadians(7.650)},
                    {130.0 - TURRET_DISTANCE_FROM_BACK, 0.2358, Math.toRadians(7.643)}
            }
    );

    private static final LinearInterpolation POWERSHOT_DATA = new LinearInterpolation(
            new double[][]{
                    {85.0 - TURRET_DISTANCE_FROM_BACK, 0.23195, Math.toRadians(7.5623)},
                    {87.0 - TURRET_DISTANCE_FROM_BACK, 0.23365, Math.toRadians(8.31850)},
                    {89.0 - TURRET_DISTANCE_FROM_BACK, 0.23325, Math.toRadians(8.31850)},
                    {92.0 - TURRET_DISTANCE_FROM_BACK, 0.23175, Math.toRadians(8.31850)},
                    {94.0 - TURRET_DISTANCE_FROM_BACK, 0.23145, Math.toRadians(8.31850)},
                    {96.0 - TURRET_DISTANCE_FROM_BACK, 0.23195, Math.toRadians(8.31850)},
                    {98.0 - TURRET_DISTANCE_FROM_BACK, 0.23325, Math.toRadians(8.31850)},
                    {100.0 - TURRET_DISTANCE_FROM_BACK, 0.23315, Math.toRadians(8.31850)},
                    {120.0 - TURRET_DISTANCE_FROM_BACK, 0.23405, Math.toRadians(11.09133)}
            }
    );

    private double distanceToTarget;
    private double angleOffset;
    private double turretError;

    private boolean forceIndex = false;

    public Shooter(Robot robot, boolean isOn, boolean isAuto) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;
        manualAngleCorrection = 0;
        manualAngleFlapCorrection = 0;

        shooterModule = new ShooterModule(robot, isOn, isAuto);
        hopperModule = new HopperModule(robot, isOn);

        modules = new Module[]{shooterModule, hopperModule};
    }

    public void toggleColour() {
        target = target.switchColour();
    }

    public void nextTarget() {
        target = target.next();
    }

    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        // aim turret
        shooterModule.manualTurret = this.manualTurret;
        shooterModule.manualPower = this.manualTurretPower;
        if (lockTarget) {
            aimTurret();
        } else {
            aimFlap();
        }

        // run flywheel if needed
        if (flywheelOn || currentTime < lastIndexTime + ShooterModule.INDEXER_RETURNED_TIME_MS + RING_FIRE_TIME) {
            shooterModule.flyWheelTargetSpeed = target.isPowershot() ? POWERSHOT_FLYWHEEL_SPEED : HIGHGOAL_FLYWHEEL_SPEED;
        } else {
            shooterModule.flyWheelTargetSpeed = 0;
        }

        // hopper, only if indexer ready
        if (queueDelivery && shooterModule.isIndexerReturned()) {
            hopperModule.deliverRings = true;
            queueDelivery = false;
        }

        // indexer logic
        handleIndexes();

        // protect regions if the wobble is up // todo move shooter outside bad zone if it's inside
        if (robot.wobbleModule.wobbleArmPosition == WobbleModule.WobbleArmPosition.AUTO_DROP || robot.wobbleModule.wobbleArmPosition == WobbleModule.WobbleArmPosition.RAISED) {
            shooterModule.upperAngleLimit = 0.75 * Math.PI;
            shooterModule.lowerAngleLimit = -0.4 * Math.PI;
            shooterModule.limitAngle = true;
        } else {
            shooterModule.limitAngle = false;
        }

        // Update both modules
        if (hopperModule.isOn()) hopperModule.update();
        if (shooterModule.isOn()) shooterModule.update();
    }

    private void aimTurret() {
        distanceToTarget = distanceToTarget(target);

        angleOffset = target.isPowershot() ? getPowershotAimValues(distanceToTarget)[1]: getHighGoalAimValues(distanceToTarget)[1];

        double turretTargetHeading = absoluteHeadingToTarget(target) - robot.drivetrain.getCurrentHeading();

        turretError = angleWrap(absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection
                - (robot.drivetrain.getCurrentHeading() + shooterModule.getCurrentTurretAngle()));

        shooterModule.setTargetTurretAngle(turretTargetHeading + angleOffset + manualAngleCorrection);

        aimFlap();
    }

    private void aimFlap() {
        distanceToTarget = distanceToTarget(target);

        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotAimValues(distanceToTarget)[0] + manualAngleFlapCorrection
                : getHighGoalAimValues(distanceToTarget)[0] + manualAngleFlapCorrection;
    }

    private void handleIndexes() {
        long currentTime = robot.getCurrentTimeMilli();

        if (queuedIndexes < 0) {
            queuedIndexes = 0;
        }

        if (queuedIndexes > 0) {
            angleOffset = target.isPowershot() ? getPowershotAimValues(distanceToTarget)[1] : getHighGoalAimValues(distanceToTarget)[1];

            turretError = angleWrap(absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection
                    - (robot.drivetrain.getCurrentHeading() + shooterModule.getCurrentTurretAngle()));

            boolean safeToIndex = hopperModule.msUntilHopperAtTurret() > ShooterModule.INDEXER_RETURNED_TIME_MS;

            if (safeToIndex && readyToIndex() && !shooterModule.indexRing) {
                shooterModule.indexRing = true;
                queuedIndexes--;

                lastIndexTime = currentTime;
            } else if (safeToIndex && forceIndex) {
                shooterModule.indexRing = true;
                forceIndex = false;
                robot.ringManager.addRingsInShooter(1);
                lastIndexTime = currentTime;
            }
        }
    }

    public boolean readyToIndex() {
        turretError = angleWrap(absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection
                - (robot.drivetrain.getCurrentHeading() + shooterModule.getCurrentTurretAngle()));

        boolean shooterReady = shooterModule.flywheelsUpToSpeed()
                && (target.isPowershot() ? Math.abs(turretError) < Math.toRadians(1.5) : Math.abs(turretError) < Math.toRadians(3))
                && Math.abs(shooterModule.getTurretVelocity()) < 0.001;
        boolean drivetrainReady = robot.drivetrain.getOdometryAngleVel() < Math.toRadians(2) // TODO tune these
                && robot.drivetrain.getOdometryVel() < 1;

        Log.v("shooter", "turreterror: " + turretError);

        return shooterReady && drivetrainReady;
    }

    /**
     * Calculate the flap angle and the offset angle required for a given distanceToTarget.
     *
     * @param distanceToTarget
     * @return A double[] of {flap angle, offsetAngle}.
     */
    private double[] getHighGoalAimValues(double distanceToTarget) {
        double[] output = HIGH_GOAL_DATA.interpolate(distanceToTarget);

        output[0] -= 0.001;
//        output[1] += 0.00;

        return output;
    }

    private double[] getPowershotAimValues(double distanceToTarget) {
        double[] output = POWERSHOT_DATA.interpolate(distanceToTarget);

        output[0] -= 0.00015;
        output[1] -= 0.0045;

        return output;
    }

    public void forceIndex() {
        forceIndex = true;
    }

    /**
     * The global heading from the robot to the target.
     *
     * @param targetGoal The target to aim at.
     * @return The global heading from the robot towards the target.
     */
    public double absoluteHeadingToTarget(ITarget targetGoal) {
        return robot.drivetrain.absoluteHeadingToPoint(targetGoal.getLocation());
    }

    public double relativeHeadingToTarget(ITarget targetGoal) {
        return robot.drivetrain.relativeAngleToPoint(targetGoal.getLocation());
    }

    public double distanceToTarget(ITarget targetGoal) {
        return robot.drivetrain.distanceToPoint(targetGoal.getLocation());
    }

    /**
     * Set the flap position of the shooter. Only has an effect if the turret isn't currently locked
     * on to a target.
     *
     * @param flapPosition The target flap position
     * @see #lockTarget
     */
    public void setFlapPosition(double flapPosition) {
        if (!lockTarget) {
            shooterModule.shooterFlapPosition = flapPosition;
        }
    }

    public void setFlyWheelTargetSpeed(double targetSpeed) {
        shooterModule.flyWheelTargetSpeed = targetSpeed;
    }

    public void setTurretTargetHeading(double targetHeading) {
        shooterModule.setTargetTurretAngle(targetHeading);
    }

    public double desiredTurretHeading() {
        angleOffset = target.isPowershot() ? getPowershotAimValues(distanceToTarget)[1] : getHighGoalAimValues(distanceToTarget)[1];

        return absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection;
    }

    public void queueIndex() {
        queuedIndexes += 1;
    }

    public void queueIndex(int numQueue) {
        queuedIndexes += numQueue;
    }

    /**
     * Add to the queue of indexes. Only has an effect if the aimbot is active.
     *
     * @param numRings The number of rings to add to the queue
     */
    public void queueIndexes(int numRings) {
        queuedIndexes += numRings;
    }

    public void clearIndexes() {
        queuedIndexes = 0;
    }

    public void deliverRings() {
        queueDelivery = true;
    }

    public boolean isFinishedIndexing() {
        return shooterModule.isFinishedIndexing() && queuedIndexes <= 0;
    }

    public boolean isFinishedFiringQueue() {
        return robot.getCurrentTimeMilli() > lastIndexTime + RING_FIRE_TIME && isFinishedIndexing();
    }

    public boolean isIndexerReturned() {
        return shooterModule.isIndexerReturned();
    }

    public double getTurretHeading() {
        return shooterModule.getCurrentTurretAngle();
    }

    public double getTurretVelocity() {
        return shooterModule.getTurretVelocity();
    }

    public HopperModule.HopperPosition getCurrentHopperPosition() {
        return hopperModule.getCurrentHopperPosition();
    }

    public boolean deliveryQueued() {
        return hopperModule.deliverRings || queueDelivery;
    }

    public ShooterModule.IndexerPosition getIndexerPosition() {
        return shooterModule.getIndexerPosition();
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Target: " + target.toString());
        data.add("Queued indexes: " + queuedIndexes + ", finished queue: " + isFinishedFiringQueue());
        data.add("Distance: " + distanceToTarget);
        data.add("angleOffset: " + angleOffset + ", manual angle offset: " + manualAngleCorrection);
        data.add("--");
        data.add("lockTarget: " + lockTarget);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
