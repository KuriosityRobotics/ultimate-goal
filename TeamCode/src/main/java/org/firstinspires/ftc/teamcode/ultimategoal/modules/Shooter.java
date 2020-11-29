package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.GoalFinder;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.ITarget;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.angleWrap;

public class Shooter extends ModuleCollection implements Module, TelemetryProvider {
    private final Robot robot;
    private boolean isOn;

    private final ShooterModule shooterModule;
    private final HopperModule hopperModule;

    // States
    public ITarget target = BLUE_HIGH;
    private ITarget oldTarget = target;
    public boolean isAimBotActive = false; // Whether or not the aimbot is actively controlling the robot.
    public int queuedIndexes = 0;

    private boolean activeToggle = false;

    // Flap angle to position constants 2.5E-03*x + 0.607
    private static final double FLAP_ANGLE_TO_POSITION_LINEAR_TERM = 0.0025;
    private static final double FLAP_ANGLE_TO_POSITION_CONSTANT_TERM = 0.607;

    // Distance to goal to angle offset constant
    // -0.0372 + 2.79E-03x + -1.31E-05x^2
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = -1.31E-05;
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 2.79E-03;
    private static final double HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = -0.0222; // -0.0372

    // 0.48 + -9.05E-03x + 5.34E-05x^2
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = 5.34E-5;
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = -9.05E-3;
    private static final double POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = 0.48;

    // Powershot distance to flap position -1.75E-04*x + 0.664
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM = 0.664;
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM = -1.75e-4;

    public double distanceSam;
    public double angleOffset;

    public double manualAngleCorrection;
    public double manualAngleFlapCorrection;

    private boolean hasSkippedForShooterSpeed = false;

    public Shooter(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;
        manualAngleCorrection = 0;

        shooterModule = new ShooterModule(robot, isOn);
        hopperModule = new HopperModule(robot, isOn);

        modules = new Module[]{shooterModule, hopperModule};
    }

    boolean weakBrakeOldState;

    public void update() {
        // Check if aimbot was toggled
        if (isAimBotActive && !activeToggle) {
            activeToggle = true;

            weakBrakeOldState = robot.drivetrain.weakBrake;

            robot.drivetrain.weakBrake = false;

            robot.drivetrain.setMovements(0, 0, 0);

            resetAiming();
        } else if (!isAimBotActive && activeToggle) {
            queuedIndexes = 0;

            if (hopperModule.isIndexerReturned()) {
                activeToggle = false;

                shooterModule.flyWheelTargetSpeed = 0;

                robot.drivetrain.weakBrake = weakBrakeOldState;

                robot.shooter.setHopperPosition(HopperModule.HopperPosition.LOWERED);
            }
        }

        if (activeToggle) {
            if (hasSkippedForShooterSpeed) {
                shooterModule.flyWheelTargetSpeed = robot.FLY_WHEEL_SPEED;
            } else {
                hasSkippedForShooterSpeed = true;
            }

            if (hopperModule.isIndexerPushed()) {
                if (oldTarget != target) {
                    resetAiming();
                    oldTarget = target;
                }

                hopperModule.hopperPosition = HopperModule.HopperPosition.RAISED;

                aimShooter(target);
                shooterModule.flyWheelTargetSpeed = Robot.FLY_WHEEL_SPEED;

                if (queuedIndexes > 0 && isCloseEnough && hopperModule.isIndexerReturned() && shooterModule.isUpToSpeed()) {
                    if (hopperModule.requestRingIndex()) {
                        queuedIndexes--;
                    }
                }
            }

            if (oldTarget != target) {
                resetAiming();
                oldTarget = target;
            }

            hopperModule.hopperPosition = HopperModule.HopperPosition.RAISED;

            aimShooter(target);

            if (queuedIndexes > 0 && isCloseEnough && hopperModule.isIndexerReturned() && shooterModule.isUpToSpeed()) {
                if (hopperModule.requestRingIndex()) {
                    queuedIndexes--;
                }
            }
        } else {
            aimFlapToTarget(target);
        }

        // Update both modules
        hopperModule.update();
        shooterModule.update();
    }

    public void resetAiming() {
        hasAlignedInitial = false;
        hasAlignedUsingVision = false;
        isDoneAiming = false;
        isCloseEnough = false;
        hasSkippedForShooterSpeed = false;

        shooterModule.flyWheelTargetSpeed = getFlyWheelTargetSpeed(); // Reset PID on shooter by temporarily setting speed to 0
    }

    public void toggleColour() {
        target = target.switchColour();
    }

    public void nextTarget() {
        target = target.next();
    }

//    /**
//     * Aim the shooter at the target specified.
//     *
//     * @param target The target to aim at.
//     */
//    public void aimShooter(ITarget target, GoalFinder.GoalLocationData loc) {
//        double distanceToTargetCenterRobot = distanceToTarget(target);
//
//        if (target.isPowershot()) {
//            angleOffset = (POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot)
//                    + (POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot)
//                    + POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
//        } else {
//            angleOffset = (HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot)
//                    + (HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot)
//                    + HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
//        }
//
//        turnToGoal(target, loc, angleOffset);
//
//        double distanceToTarget = distanceFromFlapToTarget(target, angleWrap(headingToTarget(target) + angleOffset));
//        distanceSam = distanceToTarget;
//
//        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) : getHighGoalFlapPosition(distanceToTarget);
//    }

    public void aimShooter(ITarget target) {
        double distanceToTargetCenterRobot = distanceToTarget(target);

        if (target.isPowershot()) {
            angleOffset = (POWER_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot)
                    + (POWER_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot)
                    + POWER_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
        } else {
            angleOffset = 0.9 * ((HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot)
                    + (HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot)
                    + HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM);
        }

        turnToGoal(target, angleOffset);

        double distanceToTarget = distanceFromFlapToTarget(target, angleWrap(headingToTarget(target) + angleOffset));
        distanceSam = distanceToTarget;

        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) : getHighGoalFlapPosition(distanceToTarget);
    }

    /**
     * Aim only the flap at the target. Does not attempt to move the robot.
     *
     * @param target The target to aim at.
     */
    private void aimFlapToTarget(ITarget target) {
        double distanceToTargetCenterRobot = distanceToTarget(target);
        double angleOffset = (HIGH_DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot) + (HIGH_DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot) + HIGH_DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;

        double distanceToTarget = distanceFromFlapToTarget(target, angleWrap(headingToTarget(target) + angleOffset));
        distanceSam = distanceToTarget;

        shooterModule.shooterFlapPosition = target.isPowershot() ? getPowershotFlapPosition(distanceToTarget) : getHighGoalFlapPosition(distanceToTarget);
    }

    private double getHighGoalFlapPosition(double distanceToTarget) {
//0.7188854
//        return 0.7188854
//                - (0.00123 * distanceToTarget)
//                + (0.00000567 * Math.pow(distanceToTarget, 2))
//                + (0.002 * Math.cos((6.28 * distanceToTarget - 628) / (0.00066 * Math.pow(distanceToTarget, 2) + 12)))
//                + manualAngleFlapCorrection;
        return 0.7188854 - 8500 * 1 * 0.000001
                + (-2 * 108.466 * (0.00000567 - 1 * 0.000001)) * distanceToTarget
                + (0.00000567 - 1 * 0.000001) * Math.pow(distanceToTarget, 2)
                + (0.002 * Math.cos((6.28 * distanceToTarget - 628) / (0.00066 * Math.pow(distanceToTarget, 2) + 12)))
                + manualAngleFlapCorrection;
    }

    private double getPowershotFlapPosition(double distanceToTarget) {
        return (POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM * distanceToTarget) + POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM;
    }

    /**
     * Convert an angle, in degrees, to flap position.
     *
     * @param angle Desired angle of flap servo, in degrees
     * @return Position to set servo to to achieve angle
     */
    private double flapAngleToPosition(double angle) {
        return (FLAP_ANGLE_TO_POSITION_LINEAR_TERM * angle) + FLAP_ANGLE_TO_POSITION_CONSTANT_TERM;
    }

    private double calculateAngleDelta(double yaw) {
        return Math.toDegrees(yaw) > 1 ? Math.tanh(Math.toDegrees(yaw)) : 0;
    }

    boolean hasAlignedInitial = false;
    boolean hasAlignedUsingVision = false;
    boolean isDoneAiming = false;
    boolean isCloseEnough = false;

//    private void turnToGoal(ITarget target, GoalFinder.GoalLocationData loc, double offset) {
//        if (!hasAlignedInitial) {
//            double headingToTarget = headingToTarget(target);
//
//            robot.drivetrain.setBrakeHeading(headingToTarget);
//
//            if (Math.abs(angleWrap(headingToTarget - robot.drivetrain.getCurrentHeading())) < Math.toRadians(1.5)) {
//                hasAlignedInitial = true;
//            }
//            hasAlignedInitial = true;
//        }
//
////        if (!hasAlignedUsingVision && hasAlignedInitial) {
////            if (target.isPowershot()) {
////                hasAlignedUsingVision = true; //TODO
////            } else {
////                if (loc != null) {
////                    double yawOffset = loc.getYaw();
////                    robot.drivetrain.setBrakeHeading(robot.drivetrain.getCurrentHeading() + yawOffset);
////
////                    if (yawOffset < Math.toRadians(1)) {
////                        hasAlignedUsingVision = true;
////                    }
////                } else {
////                    hasAlignedUsingVision = true;
////                }
////            }
////        }
//        hasAlignedUsingVision = true;
//
//        if (!isDoneAiming && hasAlignedUsingVision) {
//            robot.drivetrain.setBrakeHeading(robot.drivetrain.getBrakeHeading() + (offset + manualAngleCorrection));
//
//            isDoneAiming = true;
//        }
//
//        if (isDoneAiming) {
//            robot.drivetrain.setMovements(0, 0, 0);
//            isCloseEnough = Math.abs(robot.drivetrain.getCurrentHeading() - robot.drivetrain.getBrakeHeading()) < Math.toRadians(2);
//        }
//    }

    long doneAimingTime = 0;
    private void turnToGoal(ITarget target, double offset) {
        if (!hasAlignedInitial) {
            double headingToTarget = headingToTarget(target);

            robot.drivetrain.setBrakeHeading(headingToTarget);

            if (Math.abs(angleWrap(headingToTarget - robot.drivetrain.getCurrentHeading())) < Math.toRadians(2)) {
                hasAlignedInitial = true;
            }
            hasAlignedInitial = true;
        }

        hasAlignedUsingVision = true;

        if (!isDoneAiming && hasAlignedUsingVision) {
            if (target.isPowershot()) {
                robot.drivetrain.setBrakeHeading(robot.drivetrain.getBrakeHeading() + (offset * 1.015) + (manualAngleCorrection * 0.925));
            } else {
                robot.drivetrain.setBrakeHeading(robot.drivetrain.getBrakeHeading()  + (offset + manualAngleCorrection) * 0.925);
            }
            isDoneAiming = true;
            doneAimingTime = robot.getCurrentTimeMilli();
        }

        if (isDoneAiming) {
            robot.drivetrain.setMovements(0, 0, 0);

            isCloseEnough = Math.abs(robot.drivetrain.getCurrentHeading() - robot.drivetrain.getBrakeHeading()) < Math.toRadians(2);

            if (robot.getCurrentTimeMilli() > doneAimingTime + 2500) {
                isCloseEnough = true;
            }
        }
    }

    public void forceAim() {
        hasAlignedInitial = true;
        hasAlignedUsingVision = true;
        isCloseEnough = true;

        robot.drivetrain.setBrakeHeading(robot.drivetrain.getCurrentHeading());
    }

    /**
     * Calculate what heading we have to turn the robot to hit the target, incorporating vision.
     *
     * @param targetGoal The target to aim at.
     */
    public double headingToTarget(ITarget targetGoal) {
        return headingToTarget(targetGoal.getLocation());
    }

    /**
     * Calculate what heading we have to turn the robot to hit the target, incorporating vision.
     *
     * @param targetPoint The point to aim at.
     */
    public double headingToTarget(Point targetPoint) {
        Point robotPosition = robot.drivetrain.getCurrentPosition();

        double headingToTarget = angleWrap(Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y));

        return headingToTarget;
    }


    /**
     * Calculate the distance from the target goal.
     *
     * @param targetGoal The target goal.
     * @return The distance to that goal.
     */
    public double distanceFromFlapToTarget(ITarget targetGoal, double heading) {
        return distanceFromFlapToTarget(targetGoal.getLocation(), heading);
    }

    /**
     * Calculate distance from a target point.
     *
     * @param targetPoint The target point.
     * @return The distance to that point.
     */
    public double distanceFromFlapToTarget(Point targetPoint, double heading) {
        Point currentPosition = robot.drivetrain.getCurrentPosition();
        double globalAngle = Math.atan2(9.0, 5.0) - heading;
        double hypot = Math.hypot(5.0, 9.0);
        double deltaX = hypot * Math.cos(globalAngle);
        double deltaY = hypot * Math.sin(globalAngle);

        currentPosition.x += deltaX;
        currentPosition.y += deltaY;
        double distanceToTarget = Math.hypot(currentPosition.x - targetPoint.x, currentPosition.y - targetPoint.y);

        return distanceToTarget;
    }

    public double distanceToTarget(ITarget targetGoal) {
        return distanceToTarget(targetGoal.getLocation());
    }

    public double distanceToTarget(Point targetPoint) {
        Point currentPosition = robot.drivetrain.getCurrentPosition();

        double distanceToTarget = Math.hypot(currentPosition.x - targetPoint.x, currentPosition.y - targetPoint.y);

        return distanceToTarget;
    }

    /**
     * Set the flap position of the shooter. Only sets the position of the aimbot is not active.
     *
     * @param flapPosition
     * @see #isAimBotActive
     */
    public void setFlapPosition(double flapPosition) {
        if (!isAimBotActive) {
            shooterModule.shooterFlapPosition = flapPosition;
        }
    }

    /**
     * Set the flap position of the shooter. Only sets the position of the aimbot is not active.
     *
     * @param speed Target velocity of flywheels, in ticks per second.
     * @see #isAimBotActive
     */
    public void setFlyWheelSpeed(double speed) {
        if (!isAimBotActive) {
            shooterModule.flyWheelTargetSpeed = speed;
        }
    }

    public double getFlyWheelTargetSpeed() {
        return shooterModule.flyWheelTargetSpeed;
    }

    /**
     * Add one to the queue of indexes. Only has an effect if the aimbot is active.
     */
    public void queueIndexThreeRings() {
        if (isAimBotActive) {
//            queuedIndexes++;
            queuedIndexes = 3;
        }
    }

    public void queueIndex() {
        if (isAimBotActive) {
            queuedIndexes = 1;
        }
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
        return hopperModule.requestRingIndex();
    }

    public boolean isUpToSpeed() {
        return shooterModule.isUpToSpeed();
    }

    public void setHopperPosition(HopperModule.HopperPosition hopperPosition) {
        hopperModule.hopperPosition = hopperPosition;
    }

    public HopperModule.HopperPosition getHopperPosition() {
        return hopperModule.hopperPosition;
    }

    public void switchHopperPosition() {
        hopperModule.switchHopperPosition();
    }

    /**
     * Whether or not the shooter is awaiting indexes.
     *
     * @return If there are indexes queued.
     */
    public boolean isFinishedIndexing() {
        return (queuedIndexes <= 0) && hopperModule.isDoneIndexing();
    }

    public boolean isIndexerPushed() {
        return hopperModule.isIndexerPushed();
    }

    public boolean isIndexerReturned() {
        return hopperModule.isIndexerReturned();
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Is active: " + isAimBotActive);
        data.add("Active toggle: " + activeToggle);
        data.add("Target: " + target.toString());
        data.add("Queued indexes: " + queuedIndexes);
        data.add("Distance: d" + distanceSam);
        data.add("angleOffset: " + angleOffset);
        data.add("--");
        data.add("hasAlignedInitial: " + hasAlignedInitial);
        data.add("hasAlignedUsingVision: " + hasAlignedUsingVision);
        data.add("isDoneAiming: " + isDoneAiming);
        data.add("isFinishedIndexing: " + isFinishedIndexing());
        data.add("isCloseEnough: " + isCloseEnough);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}