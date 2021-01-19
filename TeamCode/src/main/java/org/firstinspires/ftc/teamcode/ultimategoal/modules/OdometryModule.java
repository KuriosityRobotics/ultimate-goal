package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.annotation.SuppressLint;
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

import java.util.ArrayList;
import java.util.Locale;

public class OdometryModule implements Module, TelemetryProvider, FileDumpProvider {
    private Robot robot;
    private boolean isOn;

    public static T265Camera slamra;

    Translation2d translation;
    Rotation2d rotation;

    // Position of the robot
    public double worldXOdometry = 0;
    public double worldYOdometry = 0;
    public double worldHeadingRadOdometry = 0;

    public double worldYVision = 0;
    public double worldXVision = 0;
    public double worldAngleVision = 0;

    Point startingPosition;

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

        this.startingPosition = startingPosition;
    }

    public void initModules() {
        // init encoders
        yLeftEncoder = robot.getDcMotor("fLeft");
        yRightEncoder = robot.getDcMotor("fRight");
        mecanumEncoder = robot.getDcMotor("bLeft");

        resetEncoders();

        // init cam
        slamra = new T265Camera(new Transform2d(new Translation2d(-0.2296668, 0), new Rotation2d(0)), 0.044, robot.hardwareMap.appContext);

        // set starting positions
        this.worldXOdometry = startingPosition.x;
        this.worldYOdometry = startingPosition.y;

        setRealsensePose(startingPosition.x, startingPosition.y, 0);

        robot.telemetry.addLine("DONE INITING T625");
    }

    public void update() {
        calculateRobotPosition();
        calculateRobotPositionT625();
    }

    public void onStart() {
        slamra.start();

        Log.d("CAMERA", "STARTING");
    }

    public void onClose() { // TODO cleanup and stuff
        slamra.stop();

        slamra.free();

//        try {
//           slamra.getClass().getMethod("cleanup").setAccessible(true);
//           slamra.getClass().getMethod("cleanup").invoke(slamra);
//        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException ex) {
//        }
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

    /**
     * side effects: sends odo data to t265
     */
    public void calculateRobotPositionT625() {
        T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();

        if (update == null) return;

        // We divide by 0.0254 to convert meters to inches
        translation = new Translation2d(update.pose.getTranslation().getX() / 0.0254, update.pose.getTranslation().getY() / 0.0254);
        rotation = update.pose.getRotation();

        worldXVision = -1 * translation.getY();
        worldYVision = translation.getX();
        worldAngleVision = -1 * rotation.getRadians();

        slamra.sendOdometry(robot.drivetrain.velocityModule.getyVel() * 0.0254, -robot.drivetrain.velocityModule.getxVel() * 0.0254);
    }

    public Point getCurrentPosition() {
        return new Point(worldXOdometry, worldYOdometry);
    }

    public Point getCurrentOdometryPosition() {
        return new Point(worldXOdometry, worldYOdometry);
    }

    /**
     * Get the current positions of the encoders.
     *
     * @return The position of the encoders in a double[], with left, right, then mecanum values.
     */
    public double[] getEncoderPositions() {
        return new double[]{leftPodKnownPosition, rightPodKnownPosition, mecanumPodKnownPosition};
    }

    public String getFileName() {
        return "odometry.txt";
    }

    public String getFileData() {
        return String.format(Locale.CANADA_FRENCH, "(%f, %f), %f", worldXOdometry, worldYOdometry, worldHeadingRadOdometry);
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

        worldXOdometry += dRobotX * Math.cos(worldHeadingRadOdometry) + dRobotY * Math.sin(worldHeadingRadOdometry);
        worldYOdometry += dRobotY * Math.cos(worldHeadingRadOdometry) - dRobotX * Math.sin(worldHeadingRadOdometry);
        //worldAngleRad =  (leftPodNewPosition - rightPodNewPosition) * INCHES_PER_ENCODER_TICK / (2 * P);
        worldHeadingRadOdometry += dTheta;
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
        worldXOdometry = x;
        worldYOdometry = y;
        worldHeadingRadOdometry = heading;

        if (slamra != null) {
            setRealsensePose(x, y, heading);
        }

        resetEncoders();
    }

    private void setRealsensePose(double x, double y, double heading) {
        slamra.setPose(new Pose2d(y * 0.0254, -x * 0.0254, new Rotation2d(heading)));
    }

    public boolean isOn() {
        return isOn;
    }

    public String getName() {
        return "OdometryModule";
    }

    public double getWorldX() {
        return worldXOdometry;
    }

    public double getWorldY() {
        return worldYOdometry;
    }

    public double getWorldHeadingRad() {
        return worldHeadingRadOdometry;
    }

    public double getWorlOdometrydHeadingRad() {
        return worldHeadingRadOdometry;
    }

    @SuppressLint("DefaultLocale")
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("worldX: " + worldXOdometry);
        data.add("worldY: " + worldYOdometry);
        data.add("heading: " + worldHeadingRadOdometry);
        data.add("worldXVision: " + worldXVision);
        data.add("worldYVision: " + worldYVision);
        data.add("headingVision: " + worldAngleVision);
        data.add(String.format("X vel: %f, y vel: %f", robot.drivetrain.velocityModule.getxVel(), robot.drivetrain.velocityModule.getyVel()));

        data.add("--");
        data.add("left: " + leftPodKnownPosition);
        data.add("right: " + rightPodKnownPosition);
        data.add("mecanum: " + mecanumPodKnownPosition);
        return data;
    }
}