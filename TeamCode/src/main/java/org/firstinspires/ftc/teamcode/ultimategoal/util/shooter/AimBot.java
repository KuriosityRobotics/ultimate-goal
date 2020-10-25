package org.firstinspires.ftc.teamcode.ultimategoal.util.shooter;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Drivetrain;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

public class AimBot {
    Robot robot;
    Drivetrain drivetrain;
    VuforiaLocalizer vuforia;

    // Position of goals, all in inches, from the origin of front blue corner (audience, left)
    private static final double HIGH_GOAL_CENTER_HEIGHT = 33.0 + (5.0 / 2) - 0.625;
    private static final double MIDDLE_GOAL_CENTER_HEIGHT = 21.0 + (12.0 / 2) - 0.625;
    private static final double LOW_GOAL_CENTER_HEIGHT = 13.0 + (8.0 / 2) - 0.625; // Subtract to account for thickness of mat
    private static final double BLUE_GOAL_CENTER_X = 23.0 + (24.0 / 2);
    private static final double RED_GOAL_CENTER_X = 23.0 + (23.5 * 3) + (24.0 / 2);
    private static final double GOAL_CENTER_Y = 6 * 24.0 - (0.5 * 2); // Full length of 6 tiles, minus .5" for edge tile's tabs.

    public AimBot(Robot robot) {
        this.robot = robot;
    }

    /**
     * Aim and shoot one ring into the target.
     *
     * @param target The target to shoot into.
     */
    public void shoot(TowerGoal target, int numRings) {
        aimShooter(target);

        robot.shooterModule.flyWheelTargetSpeed = robot.FLY_WHEEL_SPEED;

        for (int i = 0; i < numRings; i++) {
            robot.shooterModule.indexRing = true;

            while (robot.shooterModule.indexRing) {

            }
        }
    }

    /**
     * Aim the shooter at the target specified.
     *
     * @param target The target to aim at.
     */
    public void aimShooter(TowerGoal target) {
        // TODO: SET HEADING

        // Set flap
        //  -0.00000548x^2 + 0.00107x + 0.59623
        double distanceToTarget = distanceToTarget(target);
        robot.shooterModule.shooterFlapPosition = (-0.00000548 * distanceToTarget * distanceToTarget) + (0.00107 * distanceToTarget) + 0.59623;
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
        Point robotPosition = drivetrain.getCurrentPosition();

        double headingToTarget = Math.atan2(robotPosition.y - targetPoint.y, robotPosition.x - targetPoint.y);

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
        Point currentPosition = drivetrain.getCurrentPosition();

        double distanceToTarget = Math.hypot(currentPosition.x - targetPoint.x, currentPosition.y - targetPoint.y);

        // TODO: Vision correction

        return distanceToTarget;
    }
}
