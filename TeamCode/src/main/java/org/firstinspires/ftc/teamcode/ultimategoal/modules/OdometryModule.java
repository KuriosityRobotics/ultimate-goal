package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.StringHelper.concat;

public class OdometryModule implements Module, TelemetryProvider {
    private Robot robot;
    private boolean isOn;

    // Position of the robot
    public double worldX;
    public double worldY;
    public double worldAngleRad;

    // Encoders (as Motors)
    private DcMotor yLeft;
    private DcMotor yRight;
    private DcMotor mecanum;

    // Constants
    private final double INCHES_PER_ENCODER_TICK = 0.0007284406721 * 100.0 / 101.9889;
    private final double LR_ENCODER_DIST_FROM_CENTER = 6.942654509 * 3589.8638 / 3600.0 * 3531.4628211 / 3600.0;
    private final double M_ENCODER_DIST_FROM_CENTER = 4.5;

    public OdometryModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    public void init() {
        yLeft = robot.getDcMotor("fLeft");
        yRight = robot.getDcMotor("fRight");
        mecanum = robot.getDcMotor("bLeft");

        yLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        calculateRobotPosition();
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
        data.add("heading: " + worldAngleRad);
        return data;
    }

    public void fileDump() {
        robot.fileDump.addData(concat("1", " ", "odometry.txt"), concat(worldX, " ", worldY, " ", worldAngleRad));
    }

    // Helper variables
    private double leftPodKnownPosition = 0;
    private double rightPodKnownPosition = 0;
    private double mecanumPodKnownPosition = 0;

    /**
     * Calculates the robot's position.
     */
    private void calculateRobotPosition() {
        double leftPodNewPosition = yLeft.getCurrentPosition();
        double rightPodNewPosition = yRight.getCurrentPosition();
        double mecanumPodNewPosition = -1 * mecanum.getCurrentPosition();

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

        worldX += dRobotX * Math.cos(worldAngleRad) + dRobotY * Math.sin(worldAngleRad);
        worldY += dRobotY * Math.cos(worldAngleRad) - dRobotX * Math.sin(worldAngleRad);
        //worldAngleRad =  (leftPodNewPosition - rightPodNewPosition) * INCHES_PER_ENCODER_TICK / (2 * P);
        worldAngleRad += dTheta;
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

    public DcMotor getyLeft() {
        return yLeft;
    }

    public DcMotor getyRight() {
        return yRight;
    }

    public DcMotor getMecanum() {
        return mecanum;
    }

    public boolean isOn() {
        return isOn;
    }

    public String getName() {
        return "OdometryModule";
    }
}