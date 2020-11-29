package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.angleWrap;

public class Drivetrain extends ModuleCollection implements TelemetryProvider {
    Robot robot;
    public boolean isOn;

    private DrivetrainModule drivetrainModule;
    private OdometryModule odometryModule;
    public VelocityModule velocityModule;

    // States
    public boolean zeroPowerBrake = true;
    public boolean isSlowMode;
    public double xMovement = 0;
    public double yMovement = 0;
    public double turnMovement = 0;
    public boolean weakBrake = false;

    // Constants
    private final static double SLOW_MODE_FACTOR = 0.35;

    // Non-linear momentum controller factors
    private static final double NON_LINEAR_P = 0.21;
    private static final double MOMENTUM_FACTOR = 0.0017;
    private static final double INVERSE_DISTANCE_FACTOR = 0.5;
    private static final double SLOW_MOMENTUM_FACTOR = 0;

    // Non-linear angle controller factors
    private static final double TURN_NON_LINEAR_P = 1;
    private static final double TURN_MOMENTUM_FACTOR = 0;
    private static final double INVERSE_TURN_FACTOR = 0;

    public Drivetrain(Robot robot, boolean isOn) {
        this(robot, isOn, new Point(0, 0));
    }

    public Drivetrain(Robot robot, boolean isOn, Point startingPosition) {
        this.robot = robot;
        this.isOn = isOn;

        drivetrainModule = new DrivetrainModule(robot, isOn);
        odometryModule = new OdometryModule(robot, isOn, startingPosition);
        velocityModule = new VelocityModule(robot, isOn);

        modules = new Module[]{drivetrainModule, odometryModule, velocityModule};

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void update() {
        odometryModule.update();
        velocityModule.update();

        setDrivetrainMovements();

        drivetrainModule.update();
    }

    /**
     * Sets the target movements of the drivetrain.
     *
     * @param xMovement    Movement in the x (horizontal) direction.
     * @param yMovement    Movement in the y (forwards) direction.
     * @param turnMovement Movement in the rotational direction.
     */
    public void setMovements(double xMovement, double yMovement, double turnMovement) {
        this.xMovement = xMovement;
        this.yMovement = yMovement;
        this.turnMovement = turnMovement;

        if (xMovement == 0 && yMovement == 0 && turnMovement == 0 && zeroPowerBrake) {
            if (!isBrake) {
                isBrake = true;
                brakePoint = getCurrentPosition();
                brakeHeading = getCurrentHeading();
            }
        } else {
            isBrake = false;
        }
    }

    /**
     * Returns the current position of the robot.
     *
     * @return The position of the robot, as a point.
     */
    public Point getCurrentPosition() {
        return odometryModule.getCurrentPosition();
    }

    public double[] getEncoderPositions() {
        return odometryModule.getEncoderPositions();
    }

    /**
     * Returns the heading of the robot, in radians.
     *
     * @return A double in radians, of the robot's heading.
     */
    public double getCurrentHeading() {
        return odometryModule.worldAngleRad;
    }

    boolean isBrake;
    public Point brakePoint;
    public double brakeHeading;

    /**
     * Set the movements of the drivetrain according to the target movement states of this module.
     * These two movements are different when braking must be applied.
     */
    private void setDrivetrainMovements() {
        if (isBrake) {
            setMovementsToBrakePosition();
        } else {
            if (isSlowMode) {
                drivetrainModule.setMovements(xMovement * SLOW_MODE_FACTOR, yMovement * SLOW_MODE_FACTOR, turnMovement * SLOW_MODE_FACTOR);
            } else {
                drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
            }
        }
    }

    /**
     * Sets movement of drivetrain to try to stay on the brake point.
     */
    private void setMovementsToBrakePosition() {
        Point robotPosition = getCurrentPosition();
        double robotHeading = getCurrentHeading();

        double distanceToTarget = Math.hypot(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);

        if (weakBrake && distanceToTarget > .5) {
            brakePoint = new Point((brakePoint.x + robotPosition.x) / 2, (brakePoint.y + robotPosition.y) / 2);

            distanceToTarget = Math.hypot(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        }

        double absoluteAngleToTarget = Math.atan2(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        double relativeTurnAngle = angleWrap(brakeHeading - robotHeading);
        double angleError = Math.abs(relativeTurnAngle);

        if (weakBrake && angleError > .08) {
            brakeHeading = robotHeading;

            relativeTurnAngle = 0;
            angleError = 0;
        }

        double relativeAngleToPoint = absoluteAngleToTarget - robotHeading;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double totalPower = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xPower = relativeXToPoint / totalPower;
        double yPower = relativeYToPoint / totalPower;

        double xMovement = xPower;
        double yMovement = yPower;
        double turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1);

        double p = NON_LINEAR_P * Math.sqrt(distanceToTarget);

        double robotVelocity = Math.hypot(velocityModule.getxVel(), velocityModule.getyVel());
        double robotVelocityHeading = Math.atan2(velocityModule.getyVel(), velocityModule.getxVel());
        velocityAlongPath = robotVelocity * Math.cos(absoluteAngleToTarget - robotVelocityHeading);

        double inverseDistance = INVERSE_DISTANCE_FACTOR * 1 / (distanceToTarget + 0.5);
        if (isSlowMode) {
            scale = Range.clip(p - ((velocityAlongPath) * (inverseDistance) * SLOW_MOMENTUM_FACTOR), -1, 1);
        } else {
            scale = Range.clip(p - ((velocityAlongPath) * (inverseDistance) * MOMENTUM_FACTOR), -1, 1);
        }

        if (isSlowMode) {
            xMovement = Range.clip(xPower * scale, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
            yMovement = Range.clip(yPower * scale, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
        } else {
            xMovement = Range.clip(xPower * scale, -1, 1);
            yMovement = Range.clip(yPower * scale, -1, 1);
        }

        double inverseTurnAngle = INVERSE_TURN_FACTOR * 1 / (angleError + .1);
        turnScale = Range.clip((TURN_NON_LINEAR_P * ((Math.sqrt(angleError) * Math.abs(relativeTurnAngle)) / relativeTurnAngle))
                - ((velocityModule.getAngleVel()) * (inverseTurnAngle) * TURN_MOMENTUM_FACTOR), -1, 1);

        if (Math.abs(getCurrentHeading() - brakeHeading) > Math.toRadians(2) && Math.abs(turnScale) < 0.08) {
            turnScale = 0.08 * (turnScale / Math.abs(turnScale));
        }

        turnMovement = turnScale;

        if (weakBrake) {
            xMovement *= 0.65;
            yMovement *= 0.65;
            turnMovement *= 0.4;
        }

        drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
    }

    private Point lastTargetPoint = new Point();
    double velocityAlongPath;
    double scale;
    double turnScale;

    /**
     * Set the movements of the drivetrain to go to a target point. Should be called over and over
     * to adjust the movements until reaching the point.
     *
     * @param targetPoint          The target point.
     * @param moveSpeed            Speed to move.
     * @param turnSpeed            Speed to turn.
     * @param direction            The direction to face while moving.
     * @param willAngleLock        Whether or not to lock to an angle.
     * @param angleLockHeading     The angle to lock to.
     * @param isTargetingLastPoint Whether or not to activate logic specific to the last point of a path.
     * @param followRadius         The radius to follow, used for last point logic.
     */
    public void setMovementsToPoint(Point targetPoint, double moveSpeed, double turnSpeed, double direction, boolean willAngleLock, double angleLockHeading, boolean isTargetingLastPoint, double followRadius) {
        Point robotPosition = getCurrentPosition();
        double robotHeading = getCurrentHeading();

        double distanceToTarget = Math.hypot(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y);
        double absoluteAngleToTarget = Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y);

        double relativeAngleToPoint = absoluteAngleToTarget - robotHeading;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeTurnAngle = angleWrap(relativeAngleToPoint + direction);
        if (willAngleLock && isTargetingLastPoint) {
            relativeTurnAngle = angleWrap(angleLockHeading - robotHeading);
        }
        double angleError = Math.abs(relativeTurnAngle);

        double totalPower = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xPower = relativeXToPoint / totalPower;
        double yPower = relativeYToPoint / totalPower;

        double xMovement = xPower * moveSpeed;
        double yMovement = yPower * moveSpeed;
        double turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (isTargetingLastPoint) {
            double p = NON_LINEAR_P * Math.sqrt(distanceToTarget);

            double robotVelocity = Math.hypot(velocityModule.getxVel(), velocityModule.getyVel());
            double robotVelocityHeading = Math.atan2(velocityModule.getyVel(), velocityModule.getxVel());
            velocityAlongPath = robotVelocity * Math.cos(absoluteAngleToTarget - robotVelocityHeading);

            double inverseDistance = INVERSE_DISTANCE_FACTOR * 1 / (distanceToTarget + 0.5);
            if (isSlowMode) {
                scale = Range.clip(p - ((velocityAlongPath) * (inverseDistance) * SLOW_MOMENTUM_FACTOR), -1, 1);
            } else {
                scale = Range.clip(p - ((velocityAlongPath) * (inverseDistance) * MOMENTUM_FACTOR), -1, 1);
            }

            if (isSlowMode) {
                xMovement = Range.clip(((xPower * scale) > SLOW_MODE_FACTOR ? 1 : ((xPower * scale) / SLOW_MODE_FACTOR)), -moveSpeed, moveSpeed);
                yMovement = Range.clip(((yPower * scale) > SLOW_MODE_FACTOR ? 1 : ((yPower * scale) / SLOW_MODE_FACTOR)), -moveSpeed, moveSpeed);
            } else {
                xMovement = Range.clip(xPower * scale, -moveSpeed, moveSpeed);
                yMovement = Range.clip(yPower * scale, -moveSpeed, moveSpeed);
            }

            double inverseTurnAngle = INVERSE_TURN_FACTOR * 1 / (angleError + .1);
            turnScale = Range.clip((TURN_NON_LINEAR_P * ((Math.sqrt(angleError) * Math.abs(relativeTurnAngle)) / relativeTurnAngle)) - ((velocityModule.getAngleVel()) * (inverseTurnAngle) * TURN_MOMENTUM_FACTOR), -1, 1);

            turnMovement = turnScale;
        }

        setMovements(xMovement, yMovement, turnMovement);
    }

    /**
     * Move to a given point. Does not return until the drivetrain is within the threshold of the point.
     *
     * @param point point to move to.
     */
    public void moveToPoint(Point point) {
        moveToPoint(point, 1, 1, 0, false, 0);
    }

    /**
     * Move to a given point. Does not return until the drivetrain is within threshold of the point.
     *
     * @param targetPoint      The target point to move to.
     * @param moveSpeed        The speed at which to move, from 0 to 1.
     * @param turnSpeed        The speed at which to turn, from 0 to 1.
     * @param direction        The direction to follow the path.
     * @param willAngleLock    Whether or not to angle lock.
     * @param angleLockHeading The heading to angle lock to. If willAnglelock is false, this does not matter.
     */
    public void moveToPoint(Point targetPoint, double moveSpeed, double turnSpeed, double direction, boolean willAngleLock, double angleLockHeading) {
        Point robotPosition = getCurrentPosition();

        while ((Math.hypot(robotPosition.x - targetPoint.x, robotPosition.y - targetPoint.y) > PathFollow.DISTANCE_THRESHOLD)
                || (!willAngleLock || (Math.abs(angleWrap(angleLockHeading - angleLockHeading)) > PathFollow.ANGLE_THRESHOLD))) {
            setMovementsToPoint(targetPoint, moveSpeed, turnSpeed, direction, willAngleLock, angleLockHeading, false, PathFollow.FOLLOW_RADIUS);
        }
    }

    public double getDistanceToPoint(Point targetPoint) {
        Point robotPosition = getCurrentPosition();

        return Math.hypot(robotPosition.x - targetPoint.x, robotPosition.y - targetPoint.y);
    }

    public void setBrake(Point brakePoint, double brakeHeading) {
        this.brakePoint = brakePoint;
        this.brakeHeading = brakeHeading;
    }

    public void setBrakePosition(Point brakePoint) {
        this.brakePoint = brakePoint;
    }

    public void setBrakeHeading(double brakeHeading) {
        this.brakeHeading = brakeHeading;
    }

    public double getBrakeHeading() {
        return this.brakeHeading;
    }

    /**
     * DANGEROUS: Set the position of the robot
     *
     * @param x
     * @param y
     */
    public void setPosition(double x, double y, double heading) {
        odometryModule.setPosition(x, y, heading);
        velocityModule.reset();
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("xMovement: " + xMovement);
        data.add("yMovement: " + yMovement);
        data.add("turnMovement: " + turnMovement);
        data.add("isSlowMode: " + isSlowMode);
        data.add("-");
        data.add("isBrake: " + isBrake);
        data.add("Brake Point: " + brakePoint);
        data.add("Brake heading: " + brakeHeading);
        data.add("-");
        data.add("non-lin momentum scale: " + scale);
        data.add("turning scale: " + turnScale);
        return data;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
