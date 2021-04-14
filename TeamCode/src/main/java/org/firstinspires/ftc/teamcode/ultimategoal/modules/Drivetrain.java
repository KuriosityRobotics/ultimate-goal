package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain.BrakeController;
import org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain.TargetVelocityFunction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.drivetrain.VelocityPidController;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.transformToCoordinateSystem;

public class Drivetrain extends ModuleCollection implements TelemetryProvider {
    Robot robot;
    public boolean isOn;

    private final DrivetrainModule drivetrainModule;
    private final OdometryModule odometryModule;
    private final T265Module t265Module;

    // States
    public double xMovement = 0;
    public double yMovement = 0;
    public double turnMovement = 0;
    public boolean isSlowMode = false;
    public boolean zeroPowerBrake = true;
    public boolean weakBrake = false;

    // Brake states
    public boolean brake;
    public Point brakePoint;
    public double brakeHeading;

    // Constants
    private final static double SLOW_MODE_FACTOR = 0.35;
    private final static double TURN_SCALE = Math.toRadians(30);

    // Braking Controllers
    private final BrakeController towardsBrakeController = new BrakeController(
            new VelocityPidController(0.0017, 0, 0.41),
            new TargetVelocityFunction(1.77, 3, 10, 0.4),
            0.0096, 10
    );
    private final BrakeController normalToBrakeController = new BrakeController(
            new VelocityPidController(0.0064, 0, 0.2),
            new TargetVelocityFunction(0, 0, 0, 0),
            0, 1, false
    );
    private final BrakeController angularBrakeController = new BrakeController(
            new VelocityPidController(0.1, 0, 6.0),
            new TargetVelocityFunction(Math.toRadians(190), Math.toRadians(10), Math.toRadians(45), Math.toRadians(0.6)),
            0.2, Math.toRadians(110)
    );

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
        if (odometryModule.isOn()) odometryModule.update();
        if (t265Module.isOn()) t265Module.update();

        if (drivetrainModule.isOn()) {
            applyMovements();

            drivetrainModule.update();
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
            if (!brake) {
                brake = true;

                resetBrake();
            }
        } else {
            brake = false;
        }
    }

    /**
     * Set the movements of the drivetrain module according to the target movement states of this
     * module. These two movements are different when braking must be applied.
     */
    private void applyMovements() {
        if (brake) {
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

    private void adjustBrakeForWeak() {
        if (distanceToPoint(brakePoint) > 0.7) {
            Point robotPosition = getCurrentPosition();

            brakePoint = new Point((brakePoint.x + robotPosition.x) / 2, (brakePoint.y + robotPosition.y) / 2);
        }

        if (Math.abs(relativeAngleToPoint(brakePoint)) > 0.1) {
            brakeHeading = getCurrentHeading();
        }
    }

    private void resetBrake() {
        brakePoint = getCurrentPosition();
        brakeHeading = getCurrentHeading();

        towardsBrakeController.reset();
        normalToBrakeController.reset();
        angularBrakeController.reset();
    }

    /**
     * Sets movement of drivetrain to try to stay on the brake point.
     */
    private void applyMovementsToBrakePosition() {
        double[] orthPowers = calculateOrthBrakePowers();
        double angularPower = calculateAngularBrakePower();

        double xMovement = orthPowers[0];
        double yMovement = orthPowers[1];
        double turnMovement = angularPower;

        // nerf braking if weak brake
        if (weakBrake) {
            xMovement *= 0.4;
            yMovement *= 0.2;
            turnMovement *= 0.9;

//            xMovement = Math.abs(xMovement) < 0.02 ? 0 : xMovement;
//            yMovement = Math.abs(yMovement) < 0.02 ? 0 : yMovement;
        }

        if (isSlowMode) {
            xMovement = Range.clip(xMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
            yMovement = Range.clip(yMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
            turnMovement = Range.clip(turnMovement, -SLOW_MODE_FACTOR, SLOW_MODE_FACTOR);
        }

        // apply movements
        drivetrainModule.setMovements(xMovement, yMovement, turnMovement);
    }

    /**
     * Calculate the orthogonal (x, y) movements of the drivetrain in order to brake.
     *
     * @return A double[], double[0] is the x movement, double[1] is the y movement.
     */
    private double[] calculateOrthBrakePowers() {
        // todo: orth scale feedfoward using angular change?

        double[] velocitiesTowardsBrake = velocitiesTowardsPoint(brakePoint);

        double velocityTowardsBrake = velocitiesTowardsBrake[0];
        double velocityNormalToBrake = velocitiesTowardsBrake[1];

        double towardsScale = towardsBrakeController.calculatePower(distanceToPoint(brakePoint), velocityTowardsBrake);
        double normalScale = normalToBrakeController.calculatePower(0, velocityNormalToBrake);


        // Decompose desired towards target and normal to target powers into xMovement & yMovement
        Pose2d toRobotCentric = new Pose2d(new Translation2d(), new Rotation2d(-relativeAngleToPoint(brakePoint)));
        Pose2d desiredMovement = new Pose2d(normalScale, towardsScale, new Rotation2d());

        Pose2d convertedMovement = MathFunctions.transformToCoordinateSystem(toRobotCentric, desiredMovement);

//        Log.v("BRAKING", "-----------------------------------");
//        Log.v("BRAKINg", "Distance to brake: " + distanceToPoint(brakePoint));
//        Log.v("BRAKING", "Velocity towards brake: " + velocityTowardsBrake + ", Velocity normal: " + velocityNormalToBrake);
//        Log.v("BRAKING", "Target towards velo: " + towardsBrakeController.targetVelocity(distanceToPoint(brakePoint)) + ", Normal target velo: " + normalToBrakeController.targetVelocity(0));
//        Log.v("BRAKING", "Towards power: " + towardsScale + ", Normal power: " + normalScale);
//        Log.v("BRAKING", "converted x: " + convertedMovement.getTranslation().getX() + ", converted y: " + convertedMovement.getTranslation().getY());

        return new double[]{convertedMovement.getTranslation().getX(), convertedMovement.getTranslation().getY()};
    }

    private double calculateAngularBrakePower() {
        double angularVelocity = getOdometryAngleVel();

        double angleToTarget = angleWrap(brakeHeading - getCurrentHeading());

        double angularPower = angularBrakeController.calculatePower(angleToTarget, angularVelocity);

//        Log.v("BRAKING", "angleToTarget: " + angleToTarget);
//        Log.v("BRAKING", "ang velo: " + angularVelocity + ", target: " + angularBrakeController.targetVelocity(angleToTarget));
//        Log.v("BRAKING", "ang power: " + angularPower);
//        Log.v("BRAKING", "ang atbrake: " + angularBrakeController.getAtBrake());

        return angularPower;
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
     * Returns the global position of a given part on the robot.
     *
     * @param relativePartPosition The position of the part relative to the robot's center.
     * @return The position of that part, calculated using the robot's current position and
     *         heading.
     */
    public Point positionOfRobotPart(Point relativePartPosition) {
        Pose2d robotAngle = new Pose2d(new Translation2d(0, 0), new Rotation2d(getCurrentHeading()));
        Pose2d relativePartPose = new Pose2d(new Translation2d(relativePartPosition.x, relativePartPosition.y), new Rotation2d(0));

        Pose2d rotatedPart = transformToCoordinateSystem(robotAngle, relativePartPose);

        Point robotPosition = getCurrentPosition();

        return new Point(rotatedPart.getTranslation().getX() + robotPosition.x,
                rotatedPart.getTranslation().getY() + robotPosition.y);
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

    /**
     * Calculate the velocity of the robot towards a target point, by decomposing the robot's
     * velocity into two directions: towards the target point, and perpendicular to that.
     *
     * @param targetPoint
     * @return double[], double[0] is the velocity towards the point and double[1] is the velocity
     *         perpendicular to that.
     */
    public double[] velocitiesTowardsPoint(Point targetPoint) {
        double absoluteAngleToTarget = absoluteHeadingToPoint(targetPoint);

        double xVel = odometryModule.getXVel();
        double yVel = odometryModule.getYVel();

        double heading = angleWrap(Math.atan2(xVel, yVel));

        double totalVel = Math.hypot(xVel, yVel);
        double angleDiff = angleWrap(heading - absoluteAngleToTarget);

        return new double[]{totalVel * Math.cos(angleDiff), totalVel * Math.sin(angleDiff)};
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
        Point currentPosition;
        if (t265Module.isOn()) {
            Pose2d robotPose = t265Module.getRobotPose();

            Point t265Position = new Point(robotPose.getTranslation());

            if (Double.isNaN(t265Position.x) || Double.isNaN(t265Position.y)) {
                currentPosition = odometryModule.getCurrentPosition();
            } else {
                currentPosition = t265Position;
            }
        } else {
            currentPosition = odometryModule.getCurrentPosition();
        }
        return currentPosition;
    }

    /**
     * Returns the heading of the robot, in radians.
     *
     * @return A double in radians, of the robot's heading.
     */
    public double getCurrentHeading() {
        if (t265Module.isOn()) {
            return t265Module.getWorldHeadingRad();
        } else {
            return odometryModule.getWorldHeadingRad();
        }
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
        data.add("xMovement: " + xMovement + ", yMovement: " + yMovement + ", turnMovement: " + turnMovement);
        data.add("isSlowMode: " + isSlowMode);
        data.add("-");
        data.add("isBrake: " + brake);
        data.add("Brake Point: " + brakePoint + ", Brake heading: " + brakeHeading);
        return data;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }

    static class Movements {
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
