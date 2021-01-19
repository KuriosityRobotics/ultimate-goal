package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;

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
    public OdometryModule odometryModule;
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
    private final static double TURN_SCALE = Math.toRadians(30);

    // Velocity controller
    private final static double ORTH_VELOCITY_P = 0.005;
    private final static double ORTH_VELOCITY_D = 0.255;
    private final static double ANGULAR_VELOCITY_P = 0.04;

    // Velocity target constants (line with a floor, to allow for coasting)
    private final static double ORTH_VELOCITY_SLOWDOWN = 1.5; // The slope of dist vs target velocity
    private final static double ORTH_COAST_THRESHOLD = 4; // threshold to start coasting, which means hold a speed until power cutoff
    private final static double ORTH_COAST_VELOCITY = 8; // velocity to coast at
    private final static double ORTH_STOP_THRESHOLD = 0.4; // threshold at which to stop entirely (after coasting)

    private final static double ANGULAR_VELOCITY_SLOWDOWN = Math.toRadians(80);
    private final static double ANGULAR_COAST_THRESHOLD = Math.toRadians(1);
    private final static double ANGULAR_COAST_VELOCITY = 0.5;
    private final static double ANGULAR_STOP_THRESHOLD = Math.toRadians(0.5);

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

                resetBrake();
            }
        } else {
            isBrake = false;
        }
    }

    private void resetBrake() {
        brakePoint = getCurrentPosition();
        brakeHeading = getCurrentHeading();

        orthScale = 1;
        angularScale = 0;
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
            applyMovementsToBrakePosition();
        } else {
            if (isSlowMode) {
                drivetrainModule.setMovements(xMovement * SLOW_MODE_FACTOR, yMovement * SLOW_MODE_FACTOR, turnMovement * SLOW_MODE_FACTOR);
            } else {
                drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
            }
        }
    }

    double orthScale = 1;
    double angularScale = 0;

    double velocityAlongPath;
    double angularVelocity;

    double orthTargetVelocity;
    double angularTargetVelocity;

    double lastOrthVelocityError;
    double lastAngularVelocityError;

    long lastLoopTime = SystemClock.elapsedRealtime();

    /**
     * Sets movement of drivetrain to try to stay on the brake point.
     */
    private void applyMovementsToBrakePosition() {
        Point robotPosition = getCurrentPosition();
        double robotHeading = getCurrentHeading();

        double distanceToTarget = Math.hypot(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        double absoluteAngleToTarget = Math.atan2(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        double relativeTurnAngle = angleWrap(brakeHeading - robotHeading);

        if (weakBrake && distanceToTarget > 0.7) {
            brakePoint = new Point((brakePoint.x + robotPosition.x) / 2, (brakePoint.y + robotPosition.y) / 2);

            distanceToTarget = Math.hypot(brakePoint.x - robotPosition.x, brakePoint.y - robotPosition.y);
        }

        if (weakBrake && Math.abs(relativeTurnAngle) > 0.1) {
            brakeHeading = robotHeading;

            relativeTurnAngle = 0;
        }

        double relativeAngleToPoint = absoluteAngleToTarget - robotHeading;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double totalPower = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xMovement = totalPower == 0 ? 0 : relativeXToPoint / totalPower;
        double yMovement = totalPower == 0 ? 0 : relativeYToPoint / totalPower;
        double turnMovement = Range.clip(relativeTurnAngle / TURN_SCALE, -1, 1);

        // Calculate current velocity along path
        velocityAlongPath = velocityTowardsTarget(absoluteAngleToTarget);
        angularVelocity = velocityModule.getAngleVel();

        // Calculate the target velocity
        orthTargetVelocity = orthTargetVelocity(distanceToTarget);
        angularTargetVelocity = relativeTurnAngle == 0 ? 0 : angularTargetVelocity(Math.abs(relativeTurnAngle)) * (Math.abs(relativeTurnAngle) / relativeTurnAngle);

        // Maybe use last change in velocity caused by movement offset as feed forward
        // lol maybe later

        // Calculate movements
        // PID later? this might be questionable, maybe power should be directly calced instead of scaling
        long currentTime = robot.getCurrentTimeMilli();
        if (orthTargetVelocity == 0) {
            orthScale = 0;
        } else {
            double error = orthTargetVelocity - velocityAlongPath;

            double increment = error * ORTH_VELOCITY_P;

            double deriv = ((error - lastOrthVelocityError) / (currentTime - lastLoopTime)) * ORTH_VELOCITY_D;
            increment += deriv;

            orthScale = Range.clip(orthScale + increment, -1, 1);

            lastOrthVelocityError = error;
        }

        if (angularTargetVelocity == 0) {
            angularScale = 0;
        } else {
            lastAngularVelocityError = angularTargetVelocity - angularVelocity;
            double increment = lastAngularVelocityError * ANGULAR_VELOCITY_P;

            angularScale = Range.clip(angularScale + increment, -1, 1);
        }
        lastLoopTime = currentTime;

        xMovement *= orthScale;
        yMovement *= orthScale;
        turnMovement = angularScale;

        // nerf braking if weak brake
        if (weakBrake) {
            xMovement *= 0.4;
            yMovement *= 0.2;
            turnMovement *= 0.9;

            xMovement = Math.abs(xMovement) < 0.02 ? 0 : xMovement;
            yMovement = Math.abs(yMovement) < 0.02 ? 0 : yMovement;
        }

        if (isSlowMode) {
            xMovement = Range.clip(xMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
            yMovement = Range.clip(yMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
            turnMovement = Range.clip(turnMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
        }

        // apply movements
        drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
    }

    public double orthTargetVelocity(double distanceToTarget) {
        return targetVelocityFunction(distanceToTarget, ORTH_STOP_THRESHOLD, ORTH_COAST_VELOCITY, ORTH_COAST_THRESHOLD, ORTH_VELOCITY_SLOWDOWN);
    }

    public double angularTargetVelocity(double angleOffsetToTarget) {
        return targetVelocityFunction(angleOffsetToTarget, ANGULAR_STOP_THRESHOLD, ANGULAR_COAST_VELOCITY, ANGULAR_COAST_THRESHOLD, ANGULAR_VELOCITY_SLOWDOWN);
    }

    public double targetVelocityFunction(double distanceToTarget, double stopThreshold, double coastVelocity, double coastThreshold, double velocitySlowdown) {
        if (distanceToTarget < stopThreshold) {
            return 0;
        } else if (distanceToTarget < coastThreshold) {
            return coastVelocity;
        } else {
            // linear function with transformations (think back to algii/trigh)
            // use absolute value to calculate target, then add back polarity
            return velocitySlowdown * (distanceToTarget - coastThreshold) + coastVelocity;
        }
    }

    /**
     * Calculate the velocity of the robot on the path towards a target point.
     *
     * @param absoluteAngleToTarget The absolute heading from the robot to the target
     * @return The velocity of the robot towards that point
     */
    public double velocityTowardsTarget(double absoluteAngleToTarget) {
        double xVel = velocityModule.getxVel();
        double yVel = velocityModule.getyVel();

        double heading = angleWrap(Math.atan2(xVel, yVel));

        double totalVel = Math.hypot(xVel, yVel);
        double angleDiff = angleWrap(heading - absoluteAngleToTarget);

        return totalVel * Math.cos(angleDiff);
    }

    /**
     * Set the movements of the drivetrain to go to a target point. Should be called over and over
     * to adjust the movements until reaching the point.
     *
     * @param targetPoint      The target point.
     * @param moveSpeed        Speed to move.
     * @param turnSpeed        Speed to turn.
     * @param direction        The direction to face while moving.
     * @param willAngleLock    Whether or not to lock to an angle.
     * @param angleLockHeading The angle to lock to.
     */
    public void setMovementsTowardsPoint(Point targetPoint, double moveSpeed, double turnSpeed, double direction, boolean willAngleLock, double angleLockHeading) {
        Point robotPosition = getCurrentPosition();
        double robotHeading = getCurrentHeading();

        double distanceToTarget = Math.hypot(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y);
        double absoluteAngleToTarget = Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y);


        double relativeAngleToPoint = absoluteAngleToTarget - robotHeading;
        double relativeXToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        double relativeTurnAngle;
        if (willAngleLock) {
            relativeTurnAngle = angleWrap(angleLockHeading - robotHeading);
        } else {
            relativeTurnAngle = angleWrap(relativeAngleToPoint + direction);
        }

        double totalOffsetToPoint = Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint);

        double xPower = relativeXToPoint / totalOffsetToPoint;
        double yPower = relativeYToPoint / totalOffsetToPoint;

        double xMovement = xPower * moveSpeed;
        double yMovement = yPower * moveSpeed;
        double turnMovement = Range.clip(relativeTurnAngle / TURN_SCALE, -turnSpeed, turnSpeed);

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
            setMovementsTowardsPoint(targetPoint, moveSpeed, turnSpeed, direction, willAngleLock, angleLockHeading);
        }

        // TODO: FIX, use braking
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
        return odometryModule.getWorldHeadingRad();
    }

    public double getAngleVel() {
        return velocityModule.getAngleVel();
    }

    /**
     * DANGEROUS: Set the position of the robot
     *
     * @param x
     * @param y
     */
    public void setPosition(double x, double y, double heading) {
        odometryModule.setPosition(x, y, heading);
        velocityModule.reset(x, y, heading);

        brakePoint = new Point(x, y);
        brakeHeading = heading;
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
        data.add("--");
        data.add("velo along path: " + velocityAlongPath);
        data.add("target orth velo: " + orthTargetVelocity);
        data.add("angular velo: " + angularVelocity);
        data.add("target angular velo: " + angularTargetVelocity);
        data.add("orth scale: " + orthScale);
        data.add("angular scale: " + angularScale);
        data.add("last orth error: " + lastOrthVelocityError);
        return data;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
