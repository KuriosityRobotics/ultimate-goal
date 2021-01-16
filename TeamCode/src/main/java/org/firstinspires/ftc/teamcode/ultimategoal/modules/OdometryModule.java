package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDumpProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Locale;

public class OdometryModule implements Module, TelemetryProvider, FileDumpProvider {
    private Robot robot;
    private boolean isOn;

    public static T265Camera slamra;

    Translation2d translation;
    Rotation2d rotation;

    // Position of the robot
    public double worldX;
    public double worldY;
    public double worldHeadingRad;

    // Encoders (as Motors)
    private DcMotor yLeftEncoder;
    private DcMotor yRightEncoder;
    private DcMotor mecanumEncoder;

    // Constants
    private final static double INCHES_PER_ENCODER_TICK = 0.0007284406721 * 100.0 / 101.9889;
    private final static double LR_ENCODER_DIST_FROM_CENTER = 6.942654509 * 3589.8638 / 3600.0 * 3531.4628211 / 3600.0;
    private final static double M_ENCODER_DIST_FROM_CENTER = 4.5;

    public OdometryModule(Robot robot, boolean isOn) {
        this(robot, isOn, new Point(0, 0));
    }

    public OdometryModule(Robot robot, boolean isOn, Point startingPosition) {
        robot.fileDump.registerProvider(this);
        robot.telemetryDump.registerProvider(this);

        this.robot = robot;
        this.isOn = isOn;

        this.worldX = startingPosition.x;
        this.worldY = startingPosition.y;
    }

    public void initModules() {
        yLeftEncoder = robot.getDcMotor("fLeft");
        yRightEncoder = robot.getDcMotor("fRight");
        mecanumEncoder = robot.getDcMotor("bLeft");

        slamra = new T265Camera(new Transform2d(), 0.1, robot.hardwareMap.appContext);

        slamra.setPose(new Pose2d(0, 0, new Rotation2d(0)));

        robot.telemetry.addLine("DONE INITING T625");
        resetEncoders();
    }

    private void resetEncoders() {
        yLeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yRightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPodKnownPosition = 0;
        rightPodKnownPosition = 0;
        mecanumPodKnownPosition = 0;
    }

    public void onStart() {
        slamra.start();
        Log.d("CAMERA", "STARTING");
    }

    public void update() {
//        calculateRobotPosition();
        calculateRobotPositionT625();
    }

    public void onClose() { // TODO cleanup and stuff
        OdometryModule.slamra.stop();
        try {
            OdometryModule.slamra.getClass().getMethod("cleanup").setAccessible(true);
            OdometryModule.slamra.getClass().getMethod("cleanup").invoke(OdometryModule.slamra);
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException ex) {
        }
    }

    public void calculateRobotPositionT625() {
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

//        slamra.sendOdometry(0,0);
//        slamra.sendOdometry(robot.drivetrain.velocityModule.getyVel(),-1*robot.drivetrain.velocityModule.getxVel());

        // We divide by 0.0254 to convert meters to inches
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();

        worldX = -1 * translation.getY();
        worldY = translation.getX();
        worldHeadingRad = -1 * rotation.getRadians();
    }

    public Point getCurrentPosition() {
        return new Point(worldX, worldY);
    }

    /**
     * Get the current positions of the encoders.
     *
     * @return The position of the encoders in a double[], with left, right, then mecanum values.
     */
    public double[] getEncoderPositions() {
        return new double[]{leftPodKnownPosition, rightPodKnownPosition, mecanumPodKnownPosition};
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("heading: " + worldHeadingRad);
//        data.add("--");
//        data.add("left: " + leftPodKnownPosition);
//        data.add("right: " + rightPodKnownPosition);
//        data.add("mecanum: " + mecanumPodKnownPosition);
        return data;
    }

    public String getFileName() {
        return "odometry.txt";
    }

    public String getFileData() {
        return String.format(Locale.CANADA_FRENCH, "(%f, %f), %f", worldX, worldY, worldHeadingRad);
    }

    // Helper variables
    private double leftPodKnownPosition = 0;
    private double rightPodKnownPosition = 0;
    private double mecanumPodKnownPosition = 0;

    /**
     * Calculates the robot's position.
     */
    private void calculateRobotPosition() {
        double leftPodNewPosition = -yLeftEncoder.getCurrentPosition();
        double rightPodNewPosition = -yRightEncoder.getCurrentPosition();
        double mecanumPodNewPosition = mecanumEncoder.getCurrentPosition();

        double leftPodDelta = leftPodNewPosition - leftPodKnownPosition;
        double rightPodDelta = rightPodNewPosition - rightPodKnownPosition;
        double mecanumPodDelta = mecanumPodNewPosition - mecanumPodKnownPosition;

        calculateNewPosition(leftPodDelta, rightPodDelta, mecanumPodDelta);

        leftPodKnownPosition = leftPodNewPosition;
        rightPodKnownPosition = rightPodNewPosition;
        mecanumPodKnownPosition = mecanumPodNewPosition;
    }

    public void calculateNewPosition(double dLeftPod, double dRightPod, double dMecanumPod) {
        // convert all inputs to inches
        double dLeftPodInches = dLeftPod * INCHES_PER_ENCODER_TICK;
        double dRightPodInches = dRightPod * INCHES_PER_ENCODER_TICK;
        double dMecanumPodInches = dMecanumPod * INCHES_PER_ENCODER_TICK;

        // so its easier to type
        double L = dLeftPodInches;
        double R = dRightPodInches;
        double M = dMecanumPodInches;
        double P = LR_ENCODER_DIST_FROM_CENTER;
        double Q = M_ENCODER_DIST_FROM_CENTER;

        // find robot relative deltas
        double dTheta = (L - R) / (2 * P);
        double dRobotX = M * sinXOverX(dTheta) + Q * Math.sin(dTheta) - L * cosXMinusOneOverX(dTheta) + P * (Math.cos(dTheta) - 1);
        double dRobotY = L * sinXOverX(dTheta) - P * Math.sin(dTheta) + M * cosXMinusOneOverX(dTheta) + Q * (Math.cos(dTheta) - 1);

        worldX += dRobotX * Math.cos(worldHeadingRad) + dRobotY * Math.sin(worldHeadingRad);
        worldY += dRobotY * Math.cos(worldHeadingRad) - dRobotX * Math.sin(worldHeadingRad);
        //worldAngleRad =  (leftPodNewPosition - rightPodNewPosition) * INCHES_PER_ENCODER_TICK / (2 * P);
        worldHeadingRad += dTheta;
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double sinXOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = 1;
            double bottom = 1;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 2) * (2 * i + 3);
            }
            return retVal;
        } else {
            return Math.sin(x) / x;
        }
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double cosXMinusOneOverX(double x) {
        if (Math.abs(x) < 2) {
            double retVal = 0;
            double top = -x;
            double bottom = 2;
            for (int i = 0; i < 9; i++) {
                retVal += top / bottom;
                top *= -x * x;
                bottom *= (2 * i + 3) * (2 * i + 4);
            }
            return retVal;
        } else {
            return (Math.cos(x) - 1) / x;
        }
    }

    public DcMotor getyLeftEncoder() {
        return yLeftEncoder;
    }

    public DcMotor getyRightEncoder() {
        return yRightEncoder;
    }

    public DcMotor getMecanumEncoder() {
        return mecanumEncoder;
    }

    public void setPosition(double x, double y, double heading) {
        worldX = x;
        worldY = y;
        worldHeadingRad = heading;

        resetEncoders();
    }

    public boolean isOn() {
        return isOn;
    }

    public String getName() {
        return "OdometryModule";
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
}