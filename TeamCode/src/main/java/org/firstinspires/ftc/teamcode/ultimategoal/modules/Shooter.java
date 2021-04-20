package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.ITarget;

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

    public double manualAngleCorrection;
    public double manualAngleFlapCorrection;

    // Constants
    public final static int HIGHGOAL_FLYWHEEL_SPEED = 1750;
    public final static int POWERSHOT_FLYWHEEL_SPEED = 1200; // todo

    private static final double TURRET_DISTANCE_FROM_BACK = 7;

    private static final double[][] HIGH_GOAL_DATA = {
            {85.0 - TURRET_DISTANCE_FROM_BACK, 0.23, Math.toRadians(6.910)},
            {90.0 - TURRET_DISTANCE_FROM_BACK, 0.2289, Math.toRadians(6.899)},
            {95.0 - TURRET_DISTANCE_FROM_BACK, 0.2299, Math.toRadians(6.886)},
            {100.0 - TURRET_DISTANCE_FROM_BACK, 0.2285, Math.toRadians(5.951)},
            {105.0 - TURRET_DISTANCE_FROM_BACK, 0.2292, Math.toRadians(6.010)},
            {110.0 - TURRET_DISTANCE_FROM_BACK, 0.228, Math.toRadians(7.347)},
            {115.0 - TURRET_DISTANCE_FROM_BACK, 0.226, Math.toRadians(6.614)},
            {120.0 - TURRET_DISTANCE_FROM_BACK, 0.2259, Math.toRadians(5.135)},
            {125.0 - TURRET_DISTANCE_FROM_BACK, 0.2247, Math.toRadians(5.573)},
            {130.0 - TURRET_DISTANCE_FROM_BACK, 0.2266, Math.toRadians(6.507)}
    };

//    // 0.865 + -0.0184x + 1.22E-04x^2
//    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = 1.22e-04; // TODO RETUNE
//    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = -0.0184;
//    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = 0.865;

    // 0.865 + -0.0184x + 1.22E-04x^2
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = 0; // TODO RETUNE
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 0;
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = 0;

//    // 0.766 + -2.73E-03x + 1.82E-05x^2
//    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_SQUARE_TERM = 1.82e-05; // TODO RETUNE
//    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM = -2.73e-03;
//    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM = 0.763; // 0.766

    // 0.766 + -2.73E-03x + 1.82E-05x^2
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_SQUARE_TERM = 0; // TODO RETUNE
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM = 0;
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM = 0; // 0.766

    private double distanceToTarget;
    private double angleOffset;

    private int burstNum = 0;
    private boolean forceIndex = false;

    public Shooter(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;
        manualAngleCorrection = 0;

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
        if (lockTarget) {
            aimTurret();
        }

        if (flywheelOn) {
            shooterModule.flyWheelTargetSpeed = target.isPowershot() ? POWERSHOT_FLYWHEEL_SPEED : HIGHGOAL_FLYWHEEL_SPEED;
        } else {
            shooterModule.flyWheelTargetSpeed = 0;
        }

        if (shooterModule.flywheelsUpToSpeed()) {
            burstNum = 0;
        }

        if (queuedIndexes > 0) {
            boolean safeToIndex = hopperModule.msUntilHopperRaised() > ShooterModule.INDEXER_RETURNED_TIME_MS;
            boolean shooterReady = shooterModule.flywheelsUpToSpeed();

            if (safeToIndex && shooterReady && !shooterModule.indexRing) {
                shooterModule.indexRing = true;
                queuedIndexes--;
                burstNum++;
            } else if (forceIndex) {
                shooterModule.indexRing = true;
                forceIndex = false;
            }
        }

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

        shooterModule.setTargetTurretAngle(adjustedTurretTarget + angleOffset);

        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) : getHighGoalAimValues(distanceToTarget)[0];

        oldTurretTarget = turretTargetRaw;
        oldUpdateTime = currentUpdateTime;
    }

    private double getPowershotAngleOffset(double distanceToTarget) {
        return ((POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTarget * distanceToTarget)
                + (POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTarget)
                + POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM);
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
        return (POWERSHOT_DISTANCE_TO_FLAP_POSITION_SQUARE_TERM * distanceToTarget * distanceToTarget)
                + (POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM * distanceToTarget)
                + POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM;
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

    public boolean requestRingIndex() {
        if (hopperModule.getCurrentHopperPosition() != HopperModule.HopperPosition.AT_TURRET) {
            return false;
        }
        shooterModule.indexRing = true;
        return true;
    }

    public void deliverRings() {
        hopperModule.deliverRings = true;
    }

    public boolean isFinishedIndexing() {
        return shooterModule.isFinishedIndexing();
    }

    public boolean isIndexerReturned() {
        return shooterModule.isIndexerReturned();
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
        data.add("--");
        data.add("lockTarget: " + lockTarget);
        data.add("burstNumber: " + burstNum);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
