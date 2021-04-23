package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
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
    public final static int POWERSHOT_FLYWHEEL_SPEED = 1750; // todo

    private final static long RING_FIRE_TIME = 400;

    private static final double TURRET_DISTANCE_FROM_BACK = 7;

    private static final double[][] HIGH_GOAL_DATA = {
            {85.0 - TURRET_DISTANCE_FROM_BACK, 0.24, Math.toRadians(8.983)},
            {90.0 - TURRET_DISTANCE_FROM_BACK, 0.2389, Math.toRadians(8.9687)},
            {95.0 - TURRET_DISTANCE_FROM_BACK, 0.2399, Math.toRadians(8.9518)},
            {100.0 - TURRET_DISTANCE_FROM_BACK, 0.2385, Math.toRadians(7.7363)},
            {105.0 - TURRET_DISTANCE_FROM_BACK, 0.2392, Math.toRadians(7.813)},
            {110.0 - TURRET_DISTANCE_FROM_BACK, 0.238, Math.toRadians(9.5511)},
            {115.0 - TURRET_DISTANCE_FROM_BACK, 0.236, Math.toRadians(8.5982)},
            {120.0 - TURRET_DISTANCE_FROM_BACK, 0.2359, Math.toRadians(6.6755)},
            {125.0 - TURRET_DISTANCE_FROM_BACK, 0.2347, Math.toRadians(7.2449)},
            {130.0 - TURRET_DISTANCE_FROM_BACK, 0.2366, Math.toRadians(8.4591)}
    };

    private double distanceToTarget;
    private double angleOffset;
    private double turretError;

    private boolean forceIndex = false;

    public Shooter(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;
        manualAngleCorrection = 0;
        manualAngleFlapCorrection = 0;

        shooterModule = new ShooterModule(robot, isOn);
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

    long oldUpdateTime = 0;
    double oldTurretTarget = 0;

    private void aimTurret() {
        long currentUpdateTime = robot.getCurrentTimeMilli();

        distanceToTarget = distanceToTarget(target);

        angleOffset = target.isPowershot() ? getPowershotAngleOffset(distanceToTarget) : getHighGoalAimValues(distanceToTarget)[1];

        double absoluteTurretHeading = absoluteHeadingToTarget(target);

        double turretTargetRaw = absoluteTurretHeading - robot.drivetrain.getCurrentHeading();
        double turretTargetVel = 1000 * (turretTargetRaw - oldTurretTarget) / (currentUpdateTime - oldUpdateTime);

        double adjustedTurretTarget = turretTargetRaw + 0.00 * turretTargetVel;

        shooterModule.setTargetTurretAngle(adjustedTurretTarget + angleOffset + manualAngleCorrection);

        aimFlap();

        oldTurretTarget = turretTargetRaw;
        oldUpdateTime = currentUpdateTime;
    }

    private void aimFlap() {
        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) + manualAngleFlapCorrection
                : getHighGoalAimValues(distanceToTarget)[0] + manualAngleFlapCorrection;
    }

    private void handleIndexes() {
        long currentTime = robot.getCurrentTimeMilli();

        if (queuedIndexes < 0) {
            queuedIndexes = 0;
        }

        if (queuedIndexes > 0) {
            angleOffset = target.isPowershot() ? getPowershotAngleOffset(distanceToTarget) : getHighGoalAimValues(distanceToTarget)[1];

            turretError = angleWrap(absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection)
                    - angleWrap(robot.drivetrain.getCurrentHeading() + shooterModule.getCurrentTurretAngle());

            boolean safeToIndex = hopperModule.msUntilHopperRaised() > ShooterModule.INDEXER_RETURNED_TIME_MS;
            boolean shooterReady = shooterModule.flywheelsUpToSpeed()
                    && Math.abs(turretError) < Math.toRadians(1)
                    && Math.abs(shooterModule.getTurretVelocity()) < 0.008;
//            boolean drivetrainReady = Math.hypot(robot.drivetrain.getOdometryXVel(), robot.drivetrain.getOdometryYVel()) < 1
//                    && robot.drivetrain.getOdometryAngleVel() < Math.toRadians(0.1);
            boolean drivetrainReady = true;

            Log.v("shooter", "shooterready: " + shooterReady);
            Log.v("shooter", "turret error: " + turretError);

            if (safeToIndex && shooterReady && drivetrainReady && !shooterModule.indexRing) {
                shooterModule.indexRing = true;
                queuedIndexes--;

                lastIndexTime = currentTime;
            } else if (forceIndex) {
                shooterModule.indexRing = true;
                forceIndex = false;
                lastIndexTime = currentTime;
            }
        }
    }

    private double getPowershotAngleOffset(double distanceToTarget) {
        return 0.09;
    }

    /**
     * Calculate the flap angle and the offset angle required for a given distanceToTarget.
     *
     * @param distanceToTarget
     * @return A double[] of {flap angle, offsetAngle}.
     */
    private double[] getHighGoalAimValues(double distanceToTarget) { // TODO RETUNE
        // find the last datapoint w/ distance smaller than the current distance
        int distanceIndex = HIGH_GOAL_DATA.length - 2; // lower bound
        for (int i = 0; i < HIGH_GOAL_DATA.length - 1; i++) {
            if (HIGH_GOAL_DATA[i + 1][0] > distanceToTarget) {
                distanceIndex = i;
                break;
            }
        }

        double deltaD = distanceToTarget - HIGH_GOAL_DATA[distanceIndex][0];
        double flangleSlope = (HIGH_GOAL_DATA[distanceIndex + 1][1] - HIGH_GOAL_DATA[distanceIndex][1]) / (HIGH_GOAL_DATA[distanceIndex + 1][0] - HIGH_GOAL_DATA[distanceIndex][0]);
        double offsetSlope = (HIGH_GOAL_DATA[distanceIndex + 1][2] - HIGH_GOAL_DATA[distanceIndex][2]) / (HIGH_GOAL_DATA[distanceIndex + 1][0] - HIGH_GOAL_DATA[distanceIndex][0]);
        double flangle = HIGH_GOAL_DATA[distanceIndex][1] + flangleSlope * deltaD;
        double offsetAngle = HIGH_GOAL_DATA[distanceIndex][2] + offsetSlope * deltaD;

        return new double[]{flangle, offsetAngle};
    }

    private double getPowershotFlapPosition(double distanceToTarget) {
        return 0.2303;
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
        angleOffset = target.isPowershot() ? getPowershotAngleOffset(distanceToTarget) : getHighGoalAimValues(distanceToTarget)[1];

        return absoluteHeadingToTarget(target) + angleOffset + manualAngleCorrection;
    }

    /**
     * Queue three ring indexes.
     */
    public void queueIndexThreeRings() {
        queuedIndexes = 3;
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

    public HopperModule.HopperPosition targetHopperPosition() {
        return hopperModule.targetHopperPosition;
    }

    public HopperModule.HopperPosition getHopperPosition() {
        return hopperModule.getCurrentHopperPosition();
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
        data.add("Queued indexes: " + queuedIndexes);
        data.add("Distance: " + distanceToTarget);
        data.add("angleOffset: " + angleOffset);
        data.add("manual angle offset: " + manualAngleCorrection);
        data.add("--");
        data.add("lockTarget: " + lockTarget);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
