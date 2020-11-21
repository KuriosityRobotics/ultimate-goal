package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TowerGoal;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.GoalFinder;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.angleWrap;

public class Shooter implements Module, TelemetryProvider {
    private Robot robot;
    private boolean isOn;

    private ShooterModule shooterModule;

    private int queuedIndexes;

    public TowerGoal target = TowerGoal.RED_HIGH;
    public boolean isAimBotActive = false; // Whether or not the aimbot is actively controlling the robot.
    private boolean activeToggle = false;

    // Flap angle to position constants 2.5E-03*x + 0.607
    private static final double FLAP_ANGLE_TO_POSITION_LINEAR_TERM = 0.0025;
    private static final double FLAP_ANGLE_TO_POSITION_CONSTANT_TERM = 0.607;

    // Distance to goal to flap angle constants
    private static final double DISTANCE_TO_FLAP_ANGLE_SQUARE_TERM = 0.00282;
    private static final double DISTANCE_TO_FLAP_ANGLE_LINEAR_TERM = -0.615;
    private static final double DISTANCE_TO_FLAP_ANGLE_CONSTANT_TERM = 50;

    // Distance to goal to angle offset constant
    // -0.0372 + 2.79E-03x + -1.31E-05x^2
    private static final double DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM = -1.31E-05;
    private static final double DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM = 2.79E-03;
    private static final double DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM = -0.0322; // -0.0372

    // Position of goals, all in inches, from the center of the robot at the front blue corner (audience, left)
    private static final double HIGH_GOAL_CENTER_HEIGHT = 33.0 + (5.0 / 2) - 0.625;
    private static final double MIDDLE_GOAL_CENTER_HEIGHT = 21.0 + (12.0 / 2) - 0.625;
    private static final double LOW_GOAL_CENTER_HEIGHT = 13.0 + (8.0 / 2) - 0.625; // Subtract to account for thickness of mat
    //    private static final double BLUE_GOAL_CENTER_X = 23.0 + (24.0 / 2) - 9; // Subtract to account for center of robot
    private static final double BLUE_GOAL_CENTER_X = 27; // Subtract to account for center of robot
    private static final double RED_GOAL_CENTER_X = 23.0 + (23.5 * 3) + (24.0 / 2) - 9;
    //    private static final double GOAL_CENTER_Y = 6 * 24.0 - (0.5 * 2) - 9;
    private static final double GOAL_CENTER_Y = (24 * 6) - 9;

    public Shooter(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;

        shooterModule = new ShooterModule(robot, isOn);
    }

    public void init() {
        shooterModule.init();
    }

    boolean weakBrakeOldState;

    public void update() {
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
        }

        if (activeToggle) {
            robot.drivetrain.setMovements(0, 0, 0);

            aimShooter(target, robot.visionModule.getLocationData());
            //           shooterModule.flyWheelTargetSpeed = Robot.FLY_WHEEL_SPEED;

            if (queuedIndexes > 0) {
                if (shooterModule.requestRingIndex()) {
                    queuedIndexes--;
                }
            }
        }

        shooterModule.update();
    }

    double distanceToTarget;

    /**
     * Aim the shooter at the target specified.
     *
     * @param target The target to aim at.
     */
    public void aimShooter(TowerGoal target, GoalFinder.GoalLocationData loc) {
        distanceToTarget = distanceToTarget(target) - 9;
        double angleOffset = (DISTANCE_TO_ANGLE_OFFSET_SQUARE_TERM * distanceToTarget * distanceToTarget) + (DISTANCE_TO_ANGLE_OFFSET_LINEAR_TERM * distanceToTarget) + DISTANCE_TO_ANGLE_OFFSET_CONSTANT_TERM;
//        double angleOffset = 0;
        turnToGoal(target, loc, angleOffset);
        // Set flap
        double flapAngleToShoot = (DISTANCE_TO_FLAP_ANGLE_SQUARE_TERM * distanceToTarget * distanceToTarget) + (DISTANCE_TO_FLAP_ANGLE_LINEAR_TERM * distanceToTarget) + DISTANCE_TO_FLAP_ANGLE_CONSTANT_TERM;
        shooterModule.shooterFlapPosition = flapAngleToPosition(flapAngleToShoot);
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
        return yaw > 0.1 ? Math.tanh(Math.pow(yaw, 3)) : 0;
    }


    private void turnToGoal(TowerGoal target, GoalFinder.GoalLocationData loc, double offset) {
        if (loc != null)
            robot.drivetrain.setBrakeHeading(calculateAngleDelta(loc.getYaw() + offset));
        else
            robot.drivetrain.setBrakeHeading(headingToTarget(target) + offset);
    }

    /**
     * Returns the position of the given goal, relative to (0,0) being the bottom left (blue and audience side)
     * of the field.
     *
     * @param towerGoal The target goal.
     * @return The position of that goal, as a Point.
     * @see Point
     */
    public Point towerGoalPosition(TowerGoal towerGoal) {
        Point targetPoint = new Point();

        switch (towerGoal) {
            case RED_HIGH:
            case RED_LOW:
            case RED_MIDDLE:
                targetPoint = new Point(RED_GOAL_CENTER_X, GOAL_CENTER_Y);
                break;
            case BLUE_HIGH:
            case BLUE_MIDDLE:
            case BLUE_LOW:
                targetPoint = new Point(BLUE_GOAL_CENTER_X, GOAL_CENTER_Y);
                break;
        }

        return targetPoint;
    }

    /**
     * Returns the height of the given goal, relative to the top of the mats.
     *
     * @param towerGoal The target goal.
     * @return The height of that goal.
     */
    public double towerGoalHeight(TowerGoal towerGoal) {
        double height = 0;

        switch (towerGoal) {
            case RED_HIGH:
            case BLUE_HIGH:
                height = HIGH_GOAL_CENTER_HEIGHT;
                break;
            case RED_MIDDLE:
            case BLUE_MIDDLE:
                height = MIDDLE_GOAL_CENTER_HEIGHT;
                break;
            case RED_LOW:
            case BLUE_LOW:
                height = LOW_GOAL_CENTER_HEIGHT;
                break;
        }

        return height;
    }

    /**
     * Calculate what heading we have to turn the robot to hit the target, incorporating vision.
     *
     * @param targetGoal The target to aim at.
     */
    public double headingToTarget(TowerGoal targetGoal) {
        return headingToTarget(towerGoalPosition(targetGoal));
    }

    /**
     * Calculate what heading we have to turn the robot to hit the target, incorporating vision.
     *
     * @param targetPoint The point to aim at.
     */
    public double headingToTarget(Point targetPoint) {
        Point robotPosition = robot.drivetrain.getCurrentPosition();

        double headingToTarget = angleWrap(Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y));

        // TODO: vision magic for double checking

        return headingToTarget;
    }

    /**
     * Calculate the distance from the target goal.
     *
     * @param targetGoal The target goal.
     * @return The distance to that goal.
     */
    public double distanceToTarget(TowerGoal targetGoal) {
        return distanceToTarget(towerGoalPosition(targetGoal));
    }

    /**
     * Calculate distance from a target point.
     *
     * @param targetPoint The target point.
     * @return The distance to that point.
     */
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

    /**
     * Add one to the queue of indexes. Only has an effect if the aimbot is active.
     */
    public void queueRingIndex() {
        if (isAimBotActive) {
            queuedIndexes++;
        }
    }

    public void indexRing() {
        shooterModule.requestRingIndex();
    }

    /**
     * Add to the queue of indexes. Only has an effect if the aimbot is active.
     *
     * @param numRings The number of rings to add to the queue
     */
    public void queueRingIndex(int numRings) {
        queuedIndexes += numRings;
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
        data.add("Distance to target: " + distanceToTarget);
        return data;
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
