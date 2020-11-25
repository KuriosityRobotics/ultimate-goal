package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

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
    public boolean isAimBotActive = false; // Whether or not the aimbot is actively controlling the robot.
    public int queuedIndexes = 0;

    private boolean activeToggle = false;

    // Flap angle to position constants 2.5E-03*x + 0.607
    private static final double FLAP_ANGLE_TO_POSITION_LINEAR_TERM = 0.0025;
    private static final double FLAP_ANGLE_TO_POSITION_CONSTANT_TERM = 0.607;

    // Distance to goal to angle offset constant
    // -0.0372 + 2.79E-03x + -1.31E-05x^2
    private static final double DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = -1.31E-05;
    private static final double DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 2.79E-03;
    private static final double DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = -0.0222; // -0.0372

    // Powershot distance to flap position -1.75E-04*x + 0.664
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM = 0.664;
    private static final double POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM = -1.75e-4;

    public Shooter(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;

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
        } else if (!isAimBotActive && activeToggle) {
            activeToggle = false;

            queuedIndexes = 0;

            shooterModule.flyWheelTargetSpeed = 0;

            robot.drivetrain.weakBrake = weakBrakeOldState;

            robot.shooter.setHopperPosition(HopperModule.HopperPosition.LOWERED);
        }

        if (activeToggle) {
            robot.drivetrain.setMovements(0, 0, 0);

            hopperModule.hopperPosition = HopperModule.HopperPosition.RAISED;

            aimShooter(target);

            shooterModule.flyWheelTargetSpeed = robot.FLY_WHEEL_SPEED;

            if (queuedIndexes > 0) {
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

    public void toggleColour() {
        target = target.switchColour();
    }

    public void nextTarget() {
        target = target.next();
    }

    private double getHeadingForGoal(double distanceToTarget, Point targetPoint, Point robotPosition) {
        double headingToTarget = angleWrap(Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y));

        double headingOffset =
                headingToTarget
                        + (DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTarget * distanceToTarget)
                        + (DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTarget)
                        + DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
        return angleWrap(headingOffset);
    }

    public double distanceSam;

    /**
     * Aim the shooter at the target specified.
     *
     * @param target The target to aim at.
     */
    public void aimShooter(ITarget target) {
        double distanceToTargetCenterRobot = distanceToTarget(target);
        double angleOffset = (DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot) + (DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot) + DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
        robot.drivetrain.setBrakeHeading(angleWrap(headingToTarget(target) + angleOffset));

        double distanceToTarget = distanceFromFlapToTarget(target, angleWrap(headingToTarget(target) + angleOffset));
        distanceSam = distanceToTarget;

        if (target.isPowershot()) {
            aimFlapToPowershot(distanceToTarget);
        } else {
            aimFlapToHighGoal(distanceToTarget);
        }
    }

    /**
     * Aim only the flap at the target. Does not attempt to move the robot.
     *
     * @param target The target to aim at.
     */
    private void aimFlapToTarget(ITarget target) {
        double distanceToTargetCenterRobot = distanceToTarget(target);
        double angleOffset = (DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTargetCenterRobot * distanceToTargetCenterRobot) + (DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTargetCenterRobot) + DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;

        double distanceToTarget = distanceFromFlapToTarget(target, angleWrap(headingToTarget(target) + angleOffset));
        distanceSam = distanceToTarget;

        if (target.isPowershot()) {
            aimFlapToPowershot(distanceToTarget);
        } else {
            aimFlapToHighGoal(distanceToTarget);
        }
    }

    private void aimFlapToHighGoal(double distanceToTarget) {
//        double flapAngleToShoot = (DISTANCE_TO_FLAP_ANGLE_SQUARE_TERM * distanceToTarget * distanceToTarget) + (DISTANCE_TO_FLAP_ANGLE_LINEAR_TERM * distanceToTarget) + DISTANCE_TO_FLAP_ANGLE_CONSTANT_TERM;
        double flapPositionToShoot = 0.7188854 - (0.00123 * distanceToTarget) + (0.00000567 * Math.pow(distanceToTarget, 2)) + (0.002 * Math.cos((6.28 * distanceToTarget - 628) / (0.00066 * Math.pow(distanceToTarget, 2) + 12)));

        shooterModule.shooterFlapPosition = flapPositionToShoot;
    }

    private void aimFlapToPowershot(double distanceToTarget) {
        double flapPositionToShoot = (POWERSHOT_DISTANCE_TO_FLAP_POSITION_LINEAR_TERM * distanceToTarget) + POWERSHOT_DISTANCE_TO_FLAP_POSITION_CONSTANT_TERM;

        shooterModule.shooterFlapPosition = flapPositionToShoot;
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
    public void queueRingIndex() {
        if (isAimBotActive) {
//            queuedIndexes++;
            queuedIndexes = 3;
        }
    }

    public boolean requestRingIndex() {
        return hopperModule.requestRingIndex();
    }

    public boolean isUpToSpeed() {
        return shooterModule.isUpToSpeed();
    }

    /**
     * Add to the queue of indexes. Only has an effect if the aimbot is active.
     *
     * @param numRings The number of rings to add to the queue
     */
    public void queueRingIndex(int numRings) {
        queuedIndexes += numRings;
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

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Is active: " + isAimBotActive);
        data.add("Queued indexes: " + queuedIndexes);
        data.add("Distance: d" + distanceSam);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}