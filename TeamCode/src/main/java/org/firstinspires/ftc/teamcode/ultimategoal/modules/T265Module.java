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
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.util.ArrayList;

public class T265Module implements Module, TelemetryProvider {
    private Robot robot;
    private boolean isOn;

    // Camera
    public static T265Camera t265Camera;

    // Data
    private double worldX = 0;
    private double worldY = 0;
    private double worldHeadingRad = 0;

    // Velocity of the robot, in/s and rad/s
    private double xVel;
    private double yVel;
    private double angleVel;

    private Point oldWorldPosition;
    private double oldWorldAngle;
    private long oldUpdateTime;

    private double trueWorldX = 0;
    private double trueWorldY = 0;
    private double trueWorldHeadingRad = 0;

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

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        // init cam
        t265Camera = new T265Camera(new Transform2d(new Translation2d(-0.2032, 0.0762), new Rotation2d(Math.toRadians(180))), 0, robot.hardwareMap.appContext);
        t265Camera.setPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))));

        setPosition(startingPosition.getTranslation().getX(), startingPosition.getTranslation().getY(), resetOrigin.getRotation().getRadians());

        // reset helpers
        oldUpdateTime = SystemClock.elapsedRealtime();
        oldWorldAngle = 0;

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
        t265Camera.free();
    }

    /**
     * side effects: sends ogdo data to t265
     * ^removed because it will be screwed up from reset origin
     * covariance will be manually done by us
     */
    public void calculateTruePosition() {
        T265Camera.CameraUpdate update = t265Camera.getLastReceivedCameraUpdate();

        if (update == null) return;

        // We divide by INCHES_TO_METERS to convert meters to inches
        translation = new Translation2d(update.pose.getTranslation().getX() / INCHES_TO_METERS, update.pose.getTranslation().getY() / INCHES_TO_METERS);
        rotation = update.pose.getRotation();

        trueWorldX = -translation.getY();
        trueWorldY = translation.getX();
        trueWorldHeadingRad = -1 * rotation.getRadians();
    }

    private void calculateRobotPosition() {
        double originX = resetOrigin.getTranslation().getX();
        double originY = resetOrigin.getTranslation().getY();
        double originAngle = resetOrigin.getRotation().getRadians();

        double dCx = trueWorldX - originX;
        double dCy = trueWorldY - originY;

        double hypot = Math.hypot(dCx, dCy);
        double atan = Math.atan2(dCy, dCx);

        double a = atan + originAngle;

        worldX = hypot * Math.cos(a);
        worldY = hypot * Math.sin(a);
        worldHeadingRad = trueWorldHeadingRad - originAngle;
    }

    private void calculateRobotVelocity() {
        Point robotPosition = new Point(worldX, worldY);
        double robotHeading = worldHeadingRad;

        currentUpdateTime = robot.getCurrentTimeMilli();

        if (oldWorldPosition != null) {
            xVel = 1000 * (robotPosition.x - oldWorldPosition.x) / (currentUpdateTime - oldUpdateTime);
            yVel = 1000 * (robotPosition.y - oldWorldPosition.y) / (currentUpdateTime - oldUpdateTime);
            angleVel = 1000 * (robotHeading - oldWorldAngle) / (currentUpdateTime - oldUpdateTime);
        }

        oldWorldPosition = robotPosition;
        oldWorldAngle = robotHeading;
        oldUpdateTime = currentUpdateTime;
    }

    public void setPosition(double x, double y, double heading) {
        if (t265Camera != null) {
            setOriginFromPosition(x, y, heading);
            setVelocityPosition(x, y, heading);
        }
    }

    /**
     * Sets the calculated position of the T265 by moving the origin. Remembers the last set origin.
     *
     * @param x       Desired x position
     * @param y       Desired y position
     * @param heading Desired heading
     */
    private void setOriginFromPosition(double x, double y, double heading) {
        double atan = Math.atan2(y, x);
        double hypot = Math.hypot(x, y);

        double a = atan - (trueWorldHeadingRad - heading);

        double relativeX = hypot * Math.cos(a);
        double relativeY = hypot * Math.sin(a);

        resetOrigin = new Pose2d(trueWorldX - relativeX, trueWorldY - relativeY, new Rotation2d(trueWorldHeadingRad - heading));
    }

    private void setVelocityPosition(double x, double y, double heading) {
        oldWorldPosition = new Point(x, y);
        oldWorldAngle = heading;

        xVel = 0;
        yVel = 0;
        angleVel = 0;
    }

    public double getWorldX() {
        return worldX;
    }

    public double getWorldY() {
        return worldY;
    }

    public double getWorldHeadingRad() {
        return worldHeadingRad;
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

    public Point getCurrentPosition() {
        return new Point(worldX, worldY);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("trueWorldX: " + trueWorldX);
        data.add("trueWorldY: " + trueWorldY);
        data.add("trueWorldAngleDeg: " + Math.toDegrees(trueWorldHeadingRad));
        data.add("--");
        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("worldAngleDeg: " + Math.toDegrees(worldHeadingRad));
        data.add("--");
        data.add("originX: " + resetOrigin.getTranslation().getX());
        data.add("originY: " + resetOrigin.getTranslation().getY());
        data.add("originAngleDeg: " + Math.toDegrees(resetOrigin.getRotation().getRadians()));
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
