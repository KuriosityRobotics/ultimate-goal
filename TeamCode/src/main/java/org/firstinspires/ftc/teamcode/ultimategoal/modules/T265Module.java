package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions;

import java.util.ArrayList;

public class T265Module implements Module, TelemetryProvider {
    private final Robot robot;
    private final boolean isOn;

    // Camera
    public static T265Camera t265Camera;

    // Data
    private Pose2d robotPose;

    // Velocity of the robot, in/s and rad/s
    private double xVel;
    private double yVel;
    private double angleVel;

    private Pose2d oldWorldPose;
    private long oldUpdateTime;

    private static Pose2d trueRobotPose = new Pose2d();

    Pose2d startingPosition;

    private long currentUpdateTime;

    // Helpers
    private Pose2d resetOrigin;

    Translation2d translation;
    Rotation2d rotation;

    // Constants
    private static final double INCHES_TO_METERS = 0.0254;

    public T265Module(Robot robot, boolean isOn) {
        this(robot, isOn, new Pose2d(0, 0, new Rotation2d(0)));
    }

    public T265Module(Robot robot, boolean isOn, Pose2d startingPosition) {
        this.robot = robot;
        this.isOn = isOn;
        this.startingPosition = startingPosition;

        this.resetOrigin = new Pose2d(0, 0, new Rotation2d(0));
        this.robotPose = new Pose2d();

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        // init cam
        if (t265Camera == null) {
            t265Camera = new T265Camera(new Transform2d(new Translation2d(-0.2032, 0.0762), new Rotation2d(Math.toRadians(180))), 0.006, robot.hardwareMap.appContext);
            t265Camera.setPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))));
        }

        setPosition(startingPosition.getTranslation().getX(), startingPosition.getTranslation().getY(), resetOrigin.getHeading());

        // reset helpers
        oldUpdateTime = SystemClock.elapsedRealtime();
        oldWorldPose = new Pose2d();

        robot.telemetry.addLine("DONE INITING T265");
    }

    public void onStart() {
        t265Camera.start();

        Log.d("CAMERA", "STARTING");
    }

    @Override
    public void update() {
        calculateTruePosition();
        calculateRobotPosition();
        calculateRobotVelocity();
    }

    @Override
    public void onClose() {
        t265Camera.stop();
    }

    /**
     * side effects: sends odometry data to t265
     */
    public void calculateTruePosition() {
        sendOdometryData();

        T265Camera.CameraUpdate update = t265Camera.getLastReceivedCameraUpdate();

        if (update == null) return;

        // We divide by INCHES_TO_METERS to convert meters to inches
        translation = new Translation2d(update.pose.getTranslation().getX() / INCHES_TO_METERS, update.pose.getTranslation().getY() / INCHES_TO_METERS);
        rotation = update.pose.getRotation();

        trueRobotPose = new Pose2d(-translation.getY(), translation.getX(), new Rotation2d(-1 * rotation.getRadians()));
    }

    private void sendOdometryData() {
        Pose2d velocityVector = new Pose2d(robot.drivetrain.getOdometryXVel(), robot.drivetrain.getOdometryYVel(), new Rotation2d());

        Pose2d transformedVelocity = MathFunctions.transformToCoordinateSystem(new Pose2d(0, 0, new Rotation2d(-resetOrigin.getHeading()))
                , velocityVector);

        t265Camera.sendOdometry(transformedVelocity.getTranslation().getY() * INCHES_TO_METERS, -transformedVelocity.getTranslation().getX() * INCHES_TO_METERS);
    }

    private void calculateRobotPosition() {
        robotPose = MathFunctions.transformToCoordinateSystem(resetOrigin, trueRobotPose);
    }

    private void calculateRobotVelocity() {
        currentUpdateTime = robot.getCurrentTimeMilli();

        if (oldWorldPose != null) {
            xVel = 1000 * (robotPose.getTranslation().getX() - oldWorldPose.getTranslation().getX()) / (currentUpdateTime - oldUpdateTime);
            yVel = 1000 * (robotPose.getTranslation().getY() - oldWorldPose.getTranslation().getY()) / (currentUpdateTime - oldUpdateTime);
            angleVel = 1000 * (robotPose.getHeading() - oldWorldPose.getHeading()) / (currentUpdateTime - oldUpdateTime);
        }

        oldWorldPose = robotPose;
        oldUpdateTime = currentUpdateTime;
    }

    public void setPosition(double x, double y, double heading) {
        if (t265Camera != null) {
            setOriginFromPosition(x, y, heading);
            setVelocityPosition(x, y, heading);
        }
    }

    /**
     * Sets the calculated position of the T265 by moving the origin. Remembers the last set
     * origin.
     *
     * @param x       Desired x position
     * @param y       Desired y position
     * @param heading Desired heading
     */
    private void setOriginFromPosition(double x, double y, double heading) {
        double atan = Math.atan2(y, x);
        double hypot = Math.hypot(x, y);

        double a = atan - (trueRobotPose.getHeading() - heading);

        double relativeX = hypot * Math.cos(a);
        double relativeY = hypot * Math.sin(a);

        resetOrigin = new Pose2d(trueRobotPose.getTranslation().getX() - relativeX
                , trueRobotPose.getTranslation().getY() - relativeY
                , new Rotation2d(trueRobotPose.getHeading() - heading));
    }

    private void setVelocityPosition(double x, double y, double heading) {
        oldWorldPose = new Pose2d(x, y, new Rotation2d(heading));

        xVel = 0;
        yVel = 0;
        angleVel = 0;
    }

    public double getWorldHeadingRad() {
        return robotPose.getHeading();
    }

    public double getxVel() {
        return xVel;
    }

    public double getyVel() {
        return yVel;
    }

    public double getAngleVel() {
        return angleVel;
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("trueWorldX: " + trueRobotPose.getTranslation().getX());
        data.add("trueWorldY: " + trueRobotPose.getTranslation().getY());
        data.add("trueWorldAngleDeg: " + Math.toDegrees(trueRobotPose.getHeading()));
        data.add("--");
        data.add("worldX: " + robotPose.getTranslation().getX());
        data.add("worldY: " + robotPose.getTranslation().getY());
        data.add("worldAngleDeg: " + Math.toDegrees(robotPose.getHeading()));
        data.add("--");
        data.add("originX: " + resetOrigin.getTranslation().getX());
        data.add("originY: " + resetOrigin.getTranslation().getY());
        data.add("originAngleDeg: " + Math.toDegrees(resetOrigin.getHeading()));
        return data;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "T265 Module";
    }
}
