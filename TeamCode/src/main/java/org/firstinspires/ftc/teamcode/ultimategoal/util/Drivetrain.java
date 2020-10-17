package org.firstinspires.ftc.teamcode.ultimategoal.util;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.DrivetrainModule;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Module;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.OdometryModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PIDController;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.angleWrap;

public class Drivetrain implements Module, TelemetryProvider {
    Robot robot;
    public boolean isOn;

    private DrivetrainModule drivetrainModule;
    private OdometryModule odometryModule;

    PIDController pidController;

    // States
    public boolean zeroPowerBrake = true;
    public double xMovement = 0;
    public double yMovement = 0;
    public double turnMovement = 0;

    public Drivetrain(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        drivetrainModule = new DrivetrainModule(robot, isOn);
        odometryModule = new OdometryModule(robot, isOn);

        pidController = new PIDController(0.01, 0.0000001, 0, robot);

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void init() {
        drivetrainModule.init();
        odometryModule.init();
    }

    @Override
    public void update() {
        odometryModule.update();

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
//        if (zeroPowerBrake) {
//            if (xMovement == 0 && yMovement == 0 && turnMovement == 0) {
//                if (!isBrake) {
//                    brakePoint = getCurrentPosition();
//                    brakeHeading = getCurrentHeading();
//                    isBrake = true;
//                }
//
//                setMovementsToBrakePoint();
//            } else {
//                isBrake = false;
//                drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
//            }
//        } else {
        drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
//        }
    }

    /**
     * Sets movement of drivetrain to try to stay on the brake point.
     */
    public void setMovementsToBrakePoint() {
        Point robotPosition = getCurrentPosition();
        double robotHeading = getCurrentHeading();

        double distanceToTarget = Math.hypot(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        double absoluteAngleToTarget = Math.atan2(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);

        double relativeAngleToPoint = absoluteAngleToTarget - robotHeading;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeTurnAngle = angleWrap(brakeHeading - robotHeading);

        double totalPower = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xPower = relativeXToPoint / totalPower;
        double yPower = relativeYToPoint / totalPower;

        // lol p
        double xMovement = xPower;
        double yMovement = yPower;
        double turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1);

        double pidScale = pidController.calculatePID(robotPosition, brakePoint);
        xMovement *= pidScale;
        yMovement *= pidScale;

        drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
    }

    private Point lastTargetPoint = new Point();

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

        double totalPower = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xPower = relativeXToPoint / totalPower;
        double yPower = relativeYToPoint / totalPower;

        // lol p
        double xMovement = xPower * moveSpeed;
        double yMovement = yPower * moveSpeed;
        double turnMovement = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (isTargetingLastPoint) {
            xMovement *= Range.clip(distanceToTarget / followRadius, 0.25, 1);
            yMovement *= Range.clip(distanceToTarget / followRadius, 0.25, 1);
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
     * @param angleLockHeading The heading to angle lock to. If willAnglelock is false, this does not matter.r
     */
    public void moveToPoint(Point targetPoint, double moveSpeed, double turnSpeed, double direction, boolean willAngleLock, double angleLockHeading) {
        Point robotPosition = getCurrentPosition();

        while ((Math.hypot(robotPosition.x - targetPoint.x, robotPosition.y - targetPoint.y) > PathFollow.DISTANCE_THRESHOLD)
                || (!willAngleLock || (Math.abs(angleWrap(angleLockHeading - angleLockHeading)) > PathFollow.ANGLE_THRESHOLD))) {
            setMovementsToPoint(targetPoint, moveSpeed, turnSpeed, direction, willAngleLock, angleLockHeading, false, PathFollow.FOLLOW_RADIUS);
        }
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public void fileDump() {

    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("xMovement: " + xMovement);
        data.add("yMovement: " + yMovement);
        data.add("turnMovemnt: " + turnMovement);
        data.add("isBrake: " + isBrake);
        return data;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
