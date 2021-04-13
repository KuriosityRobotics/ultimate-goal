package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.ITarget;

public class Shooter extends ModuleCollection implements TelemetryProvider {
    private final Robot robot;
    private final boolean isOn;

    public final ShooterModule shooterModule;
    public final HopperModule hopperModule;

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

//    // 2.5E-03*x + 0.607
//    private static final double FLAP_ANGLE_TO_POSITION_LINEAR_TERM = 0.0025;
//    private static final double FLAP_ANGLE_TO_POSITION_CONSTANT_TERM = 0.607;

    // -0.0372 + 2.79E-03x + -1.31E-05x^2
//    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = -1.31E-05; // TODO RETUNE
//    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 2.79E-03;
//    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = -0.0222; // -0.0222
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = 0.0; // TODO RETUNE
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 0.0;
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = 0.1; // -0.0222

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

    private double distanceToGoal;
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
            boolean safeToIndex = hopperModule.msUntilHopperRaised() > shooterModule.INDEXER_RETURNED_TIME_MS;
            boolean shooterReady = burstNum > 0 || shooterModule.flywheelsUpToSpeed();

            if (safeToIndex && shooterReady && !shooterModule.indexRing) {
                shooterModule.indexRing = true;
                queuedIndexes--;
                burstNum++;
            } else if (forceIndex) {
                shooterModule.indexRing = true;
            }
        }

        // Update both modules
        hopperModule.update();
        shooterModule.update();
    }

    long oldUpdateTime = 0;
    double oldTurretTarget = 0;

    private void aimTurret() {

        long currentUpdateTime = robot.getCurrentTimeMilli();

        double distanceToTarget = distanceToTarget(target);

        angleOffset = target.isPowershot() ? getPowershotAngleOffset(distanceToTarget) : getHighGoalAngleOffset(distanceToTarget);

        double absoluteTurretHeading = absoluteHeadingToTarget(target);
        double turretTargetRaw = absoluteTurretHeading - robot.drivetrain.getCurrentHeading();

        double turretTargetVel = 1000*(turretTargetRaw - oldTurretTarget) / (currentUpdateTime - oldUpdateTime);

        double turretTargetAdj = turretTargetRaw - 0.1 * turretTargetVel;

        shooterModule.setTargetTurretAngle(turretTargetAdj);

        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) : getHighGoalFlapPosition(distanceToTarget);

        oldTurretTarget = turretTargetRaw;
        oldUpdateTime = currentUpdateTime;
    }

    private double getPowershotAngleOffset(double distanceToTarget) {
        return ((POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTarget * distanceToTarget)
                + (POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTarget)
                + POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM);
    }

    private double getHighGoalAngleOffset(double distanceToTarget) {
        return ((HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTarget * distanceToTarget)
                + (HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTarget)
                + HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM);
    }

    private double getHighGoalFlapPosition(double distanceToTarget) { // TODO RETUNE
//        if (burstNum == 1) {
//            return 0.656 + (41.1 - 0.783 * distanceToTarget + 0.00437 * Math.pow(distanceToTarget, 2)) / 1000;
//        } else if (burstNum == 2) {
//            return 0.662 + (41.1 - 0.783 * distanceToTarget + 0.00437 * Math.pow(distanceToTarget, 2)) / 1000;
//        } else {
//            return 0.7178854 - 8500 * 1 * 0.000001
//                    + (-2 * 108.466 * (0.00000567 - 1 * 0.000001)) * distanceToTarget
//                    + (0.00000567 - 1 * 0.000001) * Math.pow(distanceToTarget, 2)
//                    + (0.002 * Math.cos((6.28 * distanceToTarget - 628) / (0.00066 * Math.pow(distanceToTarget, 2) + 12)))
//                    + manualAngleFlapCorrection;
//        }
        return .23;
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

    public double getFlyWheelTargetSpeed() {
        return shooterModule.flyWheelTargetSpeed;
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

public void queueIndex( int numQueue) {
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

    public boolean flywheelsUpToSpeed() {
        return shooterModule.flywheelsUpToSpeed();
    }

    public boolean isFinishedIndexing() {
        return shooterModule.isFinishedIndexing();
    }

    public boolean isIndexerReturned() {
        return shooterModule.isIndexerReturned();
    }

    public HopperModule.HopperPosition getHopperPosition() {
        return hopperModule.getCurrentHopperPosition();
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
        data.add("Distance: " + distanceToGoal);
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
