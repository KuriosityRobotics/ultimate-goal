package org.firstinspires.ftc.teamcode.ultimategoal.modules;

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
    public double worldX = 0;
    public double worldY = 0;
    public double worldAngleRad = 0;

    private double trueWorldX = 0;
    private double trueWorldY = 0;
    private double trueWorldAngleRad = 0;

    Point startingPosition;

    // Helpers
    private Pose2d resetOrigin;

    Translation2d translation;
    Rotation2d rotation;

    // Constants
    private static final double INCHES_TO_METERS = 0.0254;

    public T265Module(Robot robot, boolean isOn) {
        this(robot, isOn, new Point(0, 0));
    }

    public T265Module(Robot robot, boolean isOn, Point startingPosition) {
        this.robot = robot;
        this.isOn = isOn;
        this.startingPosition = startingPosition;

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        // init cam
        t265Camera = new T265Camera(new Transform2d(new Translation2d(-0.2296668, 0), new Rotation2d(0)), 0.044, robot.hardwareMap.appContext);

        setRealsensePose(startingPosition.x, startingPosition.y, 0);

        robot.telemetry.addLine("DONE INITING T265");
    }

    public void onStart() {
        t265Camera.start();

        Log.d("CAMERA", "STARTING");
    }

    @Override
    public void update() {
        calculateTruePosition();
        calculateRelativePosition();
    }

    @Override
    public void onClose() {
        t265Camera.stop();
        t265Camera.free();
    }

    /**
     * side effects: sends odo data to t265
     */
    public void calculateTruePosition() {
        t265Camera.sendOdometry(robot.drivetrain.getOdometryYVel() * INCHES_TO_METERS, -robot.drivetrain.getOdometryXVel() * INCHES_TO_METERS);

        T265Camera.CameraUpdate update = t265Camera.getLastReceivedCameraUpdate();

        if (update == null) return;

        // We divide by INCHES_TO_METERS to convert meters to inches
        translation = new Translation2d(update.pose.getTranslation().getX() / INCHES_TO_METERS, update.pose.getTranslation().getY() / INCHES_TO_METERS);
        rotation = update.pose.getRotation();

        trueWorldX = -1 * translation.getY();
        trueWorldY = translation.getX();
        trueWorldAngleRad = -1 * rotation.getRadians();
    }

    private void calculateRelativePosition() {
        double originX = resetOrigin.getTranslation().getX();
        double originY = resetOrigin.getTranslation().getY();
        double originAngle = resetOrigin.getRotation().getRadians();

        double globalPositionToOrigindX = trueWorldX - originX;
        double globalPositionToOrigindY = trueWorldY - originY;

        double hypot = Math.hypot(globalPositionToOrigindX, globalPositionToOrigindY);
        double globalOriginToPositiondHeading = Math.atan2(globalPositionToOrigindY, globalPositionToOrigindX);

        double relativeHeadingFromX = globalOriginToPositiondHeading + originAngle;

        worldX = hypot * Math.cos(relativeHeadingFromX);
        worldY = hypot * Math.sin(relativeHeadingFromX);

        trueWorldAngleRad = trueWorldAngleRad - originAngle;
    }

    public void setPosition(double x, double y, double heading) {
        if (t265Camera != null) {
            setRealsensePose(x, y, heading);
        }
    }

    /**
     * Sets the calculated position of the T265 by moving the origin. Remembers the last set origin.
     *
     * @param x       Desired x position
     * @param y       Desired y position
     * @param heading Desired heading
     */
    private void setRealsensePose(double x, double y, double heading) {
        double atan = Math.atan2(y, x);
        double hypot = Math.hypot(x, y);

        double relativeAngle = atan - (trueWorldAngleRad - heading);

        double relativeX = hypot * Math.cos(relativeAngle);
        double relativeY = hypot * Math.sin(relativeAngle);

        resetOrigin = new Pose2d(trueWorldX - relativeX, trueWorldY - relativeY, new Rotation2d(trueWorldAngleRad - heading));
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("trueWorldX: " + trueWorldX);
        data.add("trueWorldY: " + trueWorldY);
        data.add("trueWorldAngleRad: " + trueWorldAngleRad);
        data.add("--");
        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("worldAngleRad: " + worldAngleRad);
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
