package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;

public class Drivetrain extends ModuleCollection implements TelemetryProvider {
    Robot robot;
    public boolean isOn;

    private DrivetrainModule drivetrainModule;
    private OdometryModule odometryModule;
    private T265Module t265Module;

    // States
    public double xMovement = 0;
    public double yMovement = 0;
    public double turnMovement = 0;
    public boolean isSlowMode = false;
    public boolean zeroPowerBrake = true;
    public boolean weakBrake = false;

    // Brake states
    public boolean isBrake;
    public Point brakePoint;
    public double brakeHeading;

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
        this(robot, isOn, new Pose2d(0, 0, new Rotation2d(0)));
    }

    public Drivetrain(Robot robot, boolean isOn, Pose2d startingPosition) {
        this.robot = robot;
        this.isOn = isOn;

        drivetrainModule = new DrivetrainModule(robot, isOn);
        odometryModule = new OdometryModule(robot, isOn, startingPosition);
        t265Module = new T265Module(robot, isOn, startingPosition);

        modules = new Module[]{drivetrainModule, odometryModule, t265Module};

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void update() {
        odometryModule.update();
        t265Module.update();

        applyMovements();

        drivetrainModule.update();
    }

    /**
     * Set the movements of the drivetrain module according to the target movement states of this
     * module. These two movements are different when braking must be applied.
     */
    private void applyMovements() {
        if (isBrake) {
            if (weakBrake) {
                adjustBrakeForWeak();
            }
            applyMovementsToBrakePosition();
        } else {
            if (isSlowMode) {
                drivetrainModule.setMovements(xMovement * SLOW_MODE_FACTOR, yMovement * SLOW_MODE_FACTOR, turnMovement * SLOW_MODE_FACTOR);
            } else {
                drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
            }
        }
    }

    /**
     * Sets the target movements of the drivetrain.
     *
     * @param movements
     */
    public void setMovements(Movements movements) {
        setMovements(movements.xMovement, movements.yMovement, movements.turnMovement);
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
        setMovements(calculateMovementsTowardsPoint(targetPoint, moveSpeed, turnSpeed, willAngleLock, angleLockHeading, direction));
    }

    /**
     * Move to a given point. Does not return until the drivetrain is within threshold of the
     * point.
     *
     * @param targetPoint      The target point to move to.
     * @param moveSpeed        The speed at which to move, from 0 to 1.
     * @param turnSpeed        The speed at which to turn, from 0 to 1.
     * @param direction        The direction to follow the path.
     * @param willAngleLock    Whether or not to angle lock.
     * @param angleLockHeading The heading to angle lock to. If willAnglelock is false, this does
     *                         not matter.
     */
    public void moveToPoint(Point targetPoint, double moveSpeed, double turnSpeed, double direction, boolean willAngleLock, double angleLockHeading) {
        // TODO: untested
        while (true) {
            Point robotPosition = getCurrentPosition();

            if ((Math.hypot(robotPosition.x - targetPoint.x, robotPosition.y - targetPoint.y) > PathFollow.DISTANCE_THRESHOLD)
                    || (!willAngleLock || (Math.abs(angleWrap(angleLockHeading - angleLockHeading)) > PathFollow.ANGLE_THRESHOLD))) {
                setMovements(0, 0, 0);
                return;
            }

            setMovementsTowardsPoint(targetPoint, moveSpeed, turnSpeed, direction, willAngleLock, angleLockHeading);
        }
    }

    /**
     * Sets the position of the robot to the given point.
     *
     * @param x
     * @param y
     */
    public void setPosition(double x, double y, double heading) {
        odometryModule.setPosition(x, y, heading);
        t265Module.setPosition(x, y, heading);

        brakePoint = new Point(x, y);
        brakeHeading = heading;
        resetBrake();
    }

    public void setBrake(Point brakePoint, double brakeHeading) {
        setBrakePosition(brakePoint);
        setBrakeHeading(brakeHeading);
    }

    public void setBrakePosition(Point brakePoint) {
        this.brakePoint = brakePoint;
    }

    public void setBrakeHeading(double brakeHeading) {
        this.brakeHeading = brakeHeading;
    }

    private void resetBrake() {
        brakePoint = getCurrentPosition();
        brakeHeading = getCurrentHeading();

        orthScale = 1;
        angularScale = 0;
    }

    private Movements calculateMovementsTowardsPoint(Point targetPoint, double moveSpeed, double turnSpeed, boolean willAngleLock, double angleLockHeading, double direction) {
        double robotHeading = getCurrentHeading();

        double distanceToTarget = distanceToPoint(targetPoint);
        double absoluteAngleToTarget = absoluteHeadingToPoint(targetPoint);

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

        return new Movements(xMovement, yMovement, turnMovement);
    }

    private void adjustBrakeForWeak() {
        if (distanceToPoint(brakePoint) > 0.7) {
            Point robotPosition = getCurrentPosition();

            brakePoint = new Point((brakePoint.x + robotPosition.x) / 2, (brakePoint.y + robotPosition.y) / 2);
        }

        if (Math.abs(relativeAngleToPoint(brakePoint)) > 0.1) {
            brakeHeading = getCurrentHeading();
        }
    }

    double orthScale = 1;
    double angularScale = 0;

    double velocityAlongPath;
    double angularVelocity;

    double orthTargetVelocity;
    double angularTargetVelocity;

    double lastOrthVelocityError;
    double angularVelocityError;

    long lastLoopTime = SystemClock.elapsedRealtime();

    /**
     * Sets movement of drivetrain to try to stay on the brake point.
     */
    private void applyMovementsToBrakePosition() {
        // Calculate current velocity along path
        velocityAlongPath = velocityTowardsPoint(brakePoint);
        angularVelocity = getOdometryAngleVel();

        // Calculate the target velocity
        orthTargetVelocity = orthTargetVelocity(distanceToPoint(brakePoint));
        angularTargetVelocity = angularTargetVelocity(relativeAngleToPoint(brakePoint));

        // Maybe use last change in velocity caused by movement offset as feed forward
        // lol maybe later

        // Calculate movements
        long currentTime = robot.getCurrentTimeMilli();

        if (orthTargetVelocity == 0) {
            orthScale = 0;
        } else {
            double error = orthTargetVelocity - velocityAlongPath;

            double proportional = error * ORTH_VELOCITY_P;
            double deriv = ((error - lastOrthVelocityError) / (currentTime - lastLoopTime)) * ORTH_VELOCITY_D;

            double increment = proportional + deriv;

            orthScale = Range.clip(orthScale + increment, -1, 1);

            lastOrthVelocityError = error;
        }

        if (angularTargetVelocity == 0) {
            angularScale = 0;
        } else {
            angularVelocityError = angularTargetVelocity - angularVelocity;

            double increment = angularVelocityError * ANGULAR_VELOCITY_P;

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

    public double distanceToPoint(Point targetPoint) {
        Point robotPosition = getCurrentPosition();

        return Math.hypot(robotPosition.x - targetPoint.x, robotPosition.y - targetPoint.y);
    }

    public double relativeAngleToPoint(Point targetPoint) {
        return angleWrap(absoluteHeadingToPoint(targetPoint) - getCurrentHeading());
    }

    public double absoluteHeadingToPoint(Point targetPoint) {
        Point robotPosition = getCurrentPosition();

        return Math.atan2(targetPoint.x - robotPosition.x, targetPoint.y - robotPosition.y);
    }

    /**
     * Calculate the velocity of the robot towards a target point.
     *
     * @param targetPoint
     * @return The velocity of the robot towards that point
     */
    public double velocityTowardsPoint(Point targetPoint) {
        double absoluteAngleToTarget = absoluteHeadingToPoint(targetPoint);

        double xVel = odometryModule.getXVel();
        double yVel = odometryModule.getYVel();

        double heading = angleWrap(Math.atan2(xVel, yVel));

        double totalVel = Math.hypot(xVel, yVel);
        double angleDiff = angleWrap(heading - absoluteAngleToTarget);

        return totalVel * Math.cos(angleDiff);
    }

    private double orthTargetVelocity(double distanceToTarget) {
        return targetVelocityFunction(distanceToTarget, ORTH_STOP_THRESHOLD, ORTH_COAST_VELOCITY, ORTH_COAST_THRESHOLD, ORTH_VELOCITY_SLOWDOWN);
    }

    private double angularTargetVelocity(double angleOffsetToTarget) {
        if (angleOffsetToTarget == 0) {
            return 0;
        }

        double sign = Math.abs(angleOffsetToTarget) / angleOffsetToTarget;

        double rawTargetVelocity = targetVelocityFunction(Math.abs(angleOffsetToTarget), ANGULAR_STOP_THRESHOLD, ANGULAR_COAST_VELOCITY, ANGULAR_COAST_THRESHOLD, ANGULAR_VELOCITY_SLOWDOWN);

        return rawTargetVelocity * sign;
    }

    private double targetVelocityFunction(double distanceToTarget, double stopThreshold, double coastVelocity, double coastThreshold, double velocitySlowdown) {
        if (distanceToTarget < stopThreshold) {
            return 0;
        } else if (distanceToTarget < coastThreshold) {
            return coastVelocity;
        } else {
            // linear function with transformations
            return velocitySlowdown * (distanceToTarget - coastThreshold) + coastVelocity;
        }
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
        Pose2d robotPose = t265Module.getRobotPose();

        return new Point(robotPose.getTranslation());
    }

    /**
     * Returns the heading of the robot, in radians.
     *
     * @return A double in radians, of the robot's heading.
     */
    public double getCurrentHeading() {
        return t265Module.getWorldHeadingRad();
    }

    public Point getCurrentOdometryPosition() {
        return odometryModule.getCurrentPosition();
    }

    public double getCurrentOdometryHeading() {
        return odometryModule.getWorldHeadingRad();
    }

    public double[] getEncoderPositions() {
        return odometryModule.getEncoderPositions();
    }

    public double getOdometryXVel() {
        return odometryModule.getXVel();
    }

    public double getOdometryYVel() {
        return odometryModule.getYVel();
    }

    public double getOdometryAngleVel() {
        return odometryModule.getAngleVel();
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
        data.add("orth scale: " + orthScale);
        data.add("angular scale: " + angularScale);
        return data;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }

    class Movements {
        public double xMovement;
        public double yMovement;
        public double turnMovement;

        public Movements(double xMovement, double yMovement, double turnMovement) {
            this.xMovement = xMovement;
            this.yMovement = yMovement;
            this.turnMovement = turnMovement;
        }
    }
}
