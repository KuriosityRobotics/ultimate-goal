package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;

public class IntakeModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean doneUnlocking;
    public double intakePower = 0;
    public boolean stopIntake;
    public IntakeBlockerPosition blockerPosition;

    // Actuators
    DcMotor intakeTop;

    Servo leftBlocker;
    Servo rightBlocker;

    // Constants
    private static final int UNLOCK_TIME = 1200;
    private static final double BLOCKER_FUNNEL_ANGLE = Math.toRadians(45);

    private static final double LEFT_BLOCKER_BLOCKING_POSITION = 0.85727 + 0.125;
    private static final double RIGHT_BLOCKER_BLOCKING_POSITION = 0.0;

    private static final double LEFT_BLOCKER_WOBBLE_POSITION = 0.88;
    private static final double RIGHT_BLOCKER_WOBBLE_POSITION = 0.09;

    private static final double LEFT_BLOCKER_OPEN_POSITION = 0.370 + 0.125;
    private static final double RIGHT_BLOCKER_OPEN_POSITION = 0.463;

    private static final double LEFT_BLOCKER_INIT_POSITION = 0.71;
    private static final double RIGHT_BLOCKER_INIT_POSITION = 0.24;

    private static final double LEFT_BLOCKER_STREAMLINE_POSITION = 0.370 + 0.125 - 0.4;
    private static final double RIGHT_BLOCKER_STREAMLINE_POSITION = 0.463 + 0.4;

    private static final double LEFT_BLOCKER_SLOPE = (LEFT_BLOCKER_OPEN_POSITION - LEFT_BLOCKER_BLOCKING_POSITION) / Math.toRadians(120);
    private static final double RIGHT_BLOCKER_SLOPE = (RIGHT_BLOCKER_OPEN_POSITION - RIGHT_BLOCKER_BLOCKING_POSITION) / Math.toRadians(120);

    private static final Point LEFT_BLOCKER_POSITION = new Point(-7.627, 11.86);
    private static final Point RIGHT_BLOCKER_POSITION = new Point(7.624, 11.86);

    // blocker is 7.623" long
    private static final double WALL_AVOID_DISTANCE = 7.623 + 5; // if blocker is within this threshold to the wall, it folds

    private static final Point[] PERIMETER_VERTICES = new Point[]{
            new Point(-9, -9),
            new Point(94 - 9, -9),
            new Point(94 - 9, 141 - 9),
            new Point(-9, 141 - 9)};

    // Helpers
    long startTime;

    public enum IntakeBlockerPosition {
        BLOCKING, FUNNEL, OPEN, WOBBLE, STREAMLINE, INIT;

        public IntakeBlockerPosition next() {
            IntakeBlockerPosition position;

            switch (name()) {
                default:
                case "BLOCKING":
                    position = FUNNEL;
                    break;
                case "FUNNEL":
                    position = BLOCKING;
                    break;
            }

            return position;
        }
    }

    public IntakeModule(Robot robot, boolean isOn, boolean isAuto) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        doneUnlocking = !isAuto;
        blockerPosition = isAuto ? IntakeModule.IntakeBlockerPosition.INIT : IntakeBlockerPosition.BLOCKING;

        stopIntake = false;
        startTime = 0;
    }

    public void initModules() {
        intakeTop = robot.getDcMotor("intakeTop");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBlocker = robot.getServo("blockerLeft");
        rightBlocker = robot.getServo("blockerRight");
    }

    public void update() {
        intakeLogic();
        intakeBlockerLogic();
    }

    boolean wasStopped = false;
    long stopTime = 0;

    private void intakeLogic() {
        long currentTime = robot.getCurrentTimeMilli();

        double power;
        if (!doneUnlocking) {
            power = 1;
            if (startTime == 0) {
                startTime = robot.getCurrentTimeMilli();
            } else if (robot.getCurrentTimeMilli() > startTime + UNLOCK_TIME) {
                doneUnlocking = true;
                power = 0;
            }
        } else {
            if (stopIntake && !wasStopped) {
                stopTime = currentTime;
            }

            if (currentTime < stopTime + 250 && intakePower == 0 && stopIntake) {
                power = -0.5;
            } else if (stopIntake) {
                power = intakePower < 0 ? intakePower : 0;
            } else {
                power = intakePower;
            }
        }
        wasStopped = stopIntake;

        runIntake(power);
    }

    private void runIntake(double power) {
        intakeTop.setPower(power);
    }

    private void intakeBlockerLogic() {
        if (!doneUnlocking) {
            leftBlocker.setPosition(LEFT_BLOCKER_INIT_POSITION);
            rightBlocker.setPosition(RIGHT_BLOCKER_INIT_POSITION);
            return;
        }

        switch (blockerPosition) {
            case INIT:
                leftBlocker.setPosition(LEFT_BLOCKER_INIT_POSITION);
                rightBlocker.setPosition(RIGHT_BLOCKER_INIT_POSITION);
                break;
            case STREAMLINE:
                leftBlocker.setPosition(LEFT_BLOCKER_STREAMLINE_POSITION);
                rightBlocker.setPosition(RIGHT_BLOCKER_STREAMLINE_POSITION);
                break;
            case BLOCKING:
                leftBlocker.setPosition(LEFT_BLOCKER_BLOCKING_POSITION);
                rightBlocker.setPosition(RIGHT_BLOCKER_BLOCKING_POSITION);
                break;
            case OPEN:
                leftBlocker.setPosition(LEFT_BLOCKER_OPEN_POSITION);
                rightBlocker.setPosition(RIGHT_BLOCKER_OPEN_POSITION);
                break;
            case WOBBLE:
                leftBlocker.setPosition(LEFT_BLOCKER_WOBBLE_POSITION);
                rightBlocker.setPosition(RIGHT_BLOCKER_WOBBLE_POSITION);
                break;
            case FUNNEL:
                if (Math.abs(MathFunctions.angleWrap(robot.drivetrain.getCurrentHeading())) < Math.toRadians(90)) {
                    double headingToHighGoal = robot.shooter.relativeHeadingToTarget(robot.shooter.target.getAllianceHigh()) - robot.drivetrain.getOdometryAngleVel() * 0.1;
                    double leftBlockerTargetAngle = Math.toRadians(90) - headingToHighGoal + BLOCKER_FUNNEL_ANGLE;
                    double rightBlockerTargetAngle = Math.toRadians(90) + headingToHighGoal + BLOCKER_FUNNEL_ANGLE;

                    double leftBlockerTarget = LEFT_BLOCKER_BLOCKING_POSITION + LEFT_BLOCKER_SLOPE * leftBlockerTargetAngle;
                    double rightBlockerTarget = RIGHT_BLOCKER_BLOCKING_POSITION + RIGHT_BLOCKER_SLOPE * rightBlockerTargetAngle;
                    leftBlocker.setPosition(leftBlockerTarget);
                    rightBlocker.setPosition(rightBlockerTarget);
                } else {
                    leftBlocker.setPosition(LEFT_BLOCKER_BLOCKING_POSITION);
                    rightBlocker.setPosition(RIGHT_BLOCKER_BLOCKING_POSITION);
                }
                break;
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Intake power: " + intakePower);
        data.add("Stop intake: " + stopIntake);
        data.add("Blocker position: " + blockerPosition.toString());
        data.add("Done unlocking: " + doneUnlocking);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
