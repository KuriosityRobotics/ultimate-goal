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
import java.util.Locale;

public class T265Module implements Module, TelemetryProvider {
    private final Robot robot;
    private final boolean isOn;

    // Camera
    public static T265Camera t265Camera;

    // Data
    private double xVel;
    private double yVel;
    private double angleVel; // rad/s

    // Helpers
    private Pose2d oldWorldPose;
    private long oldUpdateTime;

    private static Pose2d trueRobotPose;
    private static Pose2d resetOrigin;

    Pose2d startingPosition;

    // Constants
    private static final double INCHES_TO_METERS = 0.0254;

    public T265Module(Robot robot, boolean isOn, Pose2d startingPosition) {
        this.robot = robot;
        this.isOn = isOn;
        this.startingPosition = startingPosition;

        if (trueRobotPose == null) trueRobotPose = startingPosition;
        if (resetOrigin == null) resetOrigin = new Pose2d();

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        // init cam
        if (t265Camera == null) {
            t265Camera = new T265Camera(new Transform2d(new Translation2d(-0.0285496, 0.2097278), new Rotation2d(Math.toRadians(270))), 0.0055, robot.hardwareMap.appContext);
            t265Camera.setPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))));
        }

        try {
            t265Camera.start();

            Log.d("CAMERA", "STARTING");
        } catch (Exception e) {
            e.printStackTrace();
        }

        // reset helpers
        oldUpdateTime = SystemClock.elapsedRealtime();
        oldWorldPose = new Pose2d();

        robot.telemetry.addLine("DONE INITING T265");
    }

    public void onStart() {
        try {
            t265Camera.start();

            Log.d("CAMERA", "STARTING");
        } catch (Exception e) {
            e.printStackTrace();
        }

        calculateTruePosition();

        setPosition(startingPosition.getTranslation().getX(), startingPosition.getTranslation().getY(), resetOrigin.getHeading());
    }

    @Override
    public void update() {
        sendOdometryData();
        calculateTruePosition();
        getRobotPose();
        calculateRobotVelocity();
    }

    @Override
    public void onClose() {
//        t265Camera.stop();
    }

    public static void calculateTruePosition() {
        T265Camera.CameraUpdate update = t265Camera.getLastReceivedCameraUpdate();

        if (update == null) return;

        // We divide by INCHES_TO_METERS to convert meters to inches
        Translation2d translation = new Translation2d(update.pose.getTranslation().getX() / INCHES_TO_METERS, update.pose.getTranslation().getY() / INCHES_TO_METERS);
        Rotation2d rotation = update.pose.getRotation();

        trueRobotPose = new Pose2d(-translation.getY(), translation.getX(), new Rotation2d(-1 * rotation.getRadians()));
    }

    public static void safeRefreshPosition() {
        if (t265Camera != null) {
            calculateTruePosition();
        }
    }

    private void sendOdometryData() {
        Pose2d velocityVector = new Pose2d(robot.drivetrain.getOdometryXVel(), robot.drivetrain.getOdometryYVel(), new Rotation2d());

        Pose2d transformedVelocity = MathFunctions.transformToCoordinateSystem(new Pose2d(0, 0, new Rotation2d(-resetOrigin.getHeading()))
                , velocityVector);

        t265Camera.sendOdometry(transformedVelocity.getTranslation().getY() * INCHES_TO_METERS, -transformedVelocity.getTranslation().getX() * INCHES_TO_METERS);
    }

    private void calculateRobotVelocity() {
        long currentUpdateTime = robot.getCurrentTimeMilli();

        if (oldWorldPose != null) {
            xVel = 1000 * (trueRobotPose.getTranslation().getX() - oldWorldPose.getTranslation().getX()) / (currentUpdateTime - oldUpdateTime);
            yVel = 1000 * (trueRobotPose.getTranslation().getY() - oldWorldPose.getTranslation().getY()) / (currentUpdateTime - oldUpdateTime);
            angleVel = 1000 * (trueRobotPose.getHeading() - oldWorldPose.getHeading()) / (currentUpdateTime - oldUpdateTime);
        }

        oldWorldPose = trueRobotPose;
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

    public static Pose2d getRobotPose() {
        if (trueRobotPose == null || resetOrigin == null) {
            return null;
        } else {
            return MathFunctions.transformToCoordinateSystem(resetOrigin, trueRobotPose);
        }
    }

    public double getWorldHeadingRad() {
        return trueRobotPose.getRotation().getRadians() - resetOrigin.getRotation().getRadians();
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

    @Override
    public ArrayList<String> getTelemetryData() {
        Pose2d robotPose = getRobotPose();

        ArrayList<String> data = new ArrayList<>();
        data.add(String.format(Locale.ROOT, "trueWorldX: %.4f, trueWorldY: %.4f", trueRobotPose.getTranslation().getX(), trueRobotPose.getTranslation().getY()));
        data.add("trueWorldAngleDeg: " + Math.toDegrees(trueRobotPose.getHeading()));
        data.add("--");
        data.add("worldX: " + robotPose.getTranslation().getX());
        data.add("worldY: " + robotPose.getTranslation().getY());
        data.add("worldAngleDeg: " + Math.toDegrees(robotPose.getHeading()));
        data.add("--");
        data.add(String.format(Locale.ROOT, "originX: %.4f, originY: %.4f", resetOrigin.getTranslation().getX(), resetOrigin.getTranslation().getY()));
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
