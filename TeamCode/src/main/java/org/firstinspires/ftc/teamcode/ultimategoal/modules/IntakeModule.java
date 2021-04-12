package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.wrappers.AnalogDistance;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.lineSegmentPointDistance;

public class IntakeModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean doneUnlocking;
    public double intakePower = 0;
    public IntakeBlockerPosition blockerPosition;

    // Actuators
    DcMotor intakeTop;
    DcMotor intakeBottom;

    Servo leftBlocker;
    Servo rightBlocker;

    // Sensors
    AnalogInput intakeDistance;

    // Constants
    private static final int UNLOCK_TIME = 1000;
    private static final double BLOCKER_FUNNEL_ANGLE = Math.toRadians(45);

    private static final Point LEFT_BLOCKER_POSITION = new Point(-7.627, 11.86);
    private static final Point RIGHT_BLOCKER_POSITION = new Point(7.624, 11.86);

    // blocker is 7.623" long
    private static final double WALL_AVOID_DISTANCE = 7.623 + 5; // if blocker is within this threshold to the wall, it folds

    private static final Point[] LEFT_BLOCKER_KNOWN_SERVO_POSITIONS = new Point[]{new Point(0, 0.85727), new Point(Math.toRadians(90), 0.492)};
    private static final Point[] RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS = new Point[]{new Point(0, 0.0), new Point(Math.toRadians(90), 0.347)};

    private static final Point[] PERIMETER_VERTICES = new Point[]{
            new Point(-9, -9),
            new Point(94 - 9, -9),
            new Point(94 - 9, 141 - 9),
            new Point(-9, 141 - 9)};

    // Helpers
    boolean holdRing;

    long startTime = 0;

    public enum IntakeBlockerPosition {
        BLOCKING, OPEN;

        public IntakeBlockerPosition next() {
            IntakeBlockerPosition position;

            switch(name()) {
                default:
                case "BLOCKING":
                    position = OPEN;
                    break;
                case "OPEN":
                    position = BLOCKING;
                    break;
            }

            return position;
        }
    }

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        this.timeLastPass = robot.getCurrentTimeMilli();

        doneUnlocking = false;
        blockerPosition = IntakeBlockerPosition.BLOCKING;

        holdRing = false;
        startTime = 0;
    }

    public void initModules() {
        intakeDistance = robot.hardwareMap.get(AnalogInput.class, "distance");

        intakeTop = robot.getDcMotor("intakeTop");
        intakeBottom = robot.getDcMotor("intakeBottom");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBlocker = robot.getServo("blockerLeft");
        rightBlocker = robot.getServo("blockerRight");
    }




    public void tryToSetIntakePower(double desired) {
        if(desired > 0) { // Intake iff total amount of rings on robot is less than three.
            if(robot.shooter.turretModule.currentRingsInTurret + robot.shooter.hopperModule.getRingsInHopper() < 3)
                this.intakePower = desired;
            else
                this.intakePower = 0;
        } else if (desired <= 0){ // Allow outtake always
            this.intakePower = desired;
        }
    }


    private double timeLastPass;


    boolean seenRing = false;
    int distanceSensorPasses = 0;

    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        double voltage = intakeDistance.getVoltage();
        if (Math.abs(2.2 - voltage) < Math.abs(1.1 - voltage)) {
            if (!seenRing)
                seenRing = true; // we need to track when the sensor starts seeing it, and when it finishes

        } else if (seenRing) {
            if (robot.intakeModule.intakeBottom.getPower() > 0) // outtaking or intaking ?
                distanceSensorPasses = distanceSensorPasses + 1;
            else
                distanceSensorPasses = distanceSensorPasses - 1;
            seenRing = false;

        }
        if (currentTime - timeLastPass > 200 && (distanceSensorPasses & 1) == 1 && seenRing) { // emergency in case it somehow gets out of sync:  timeout after two seconds & round up
            distanceSensorPasses++;
        }
        timeLastPass = currentTime;

        intakeLogic();
        intakeBlockerLogic();
    }

    private void intakeLogic() {
        if (!doneUnlocking) {
            double power = 1;
            if (startTime == 0) {
                startTime = robot.getCurrentTimeMilli();
            } else if (robot.getCurrentTimeMilli() > startTime + UNLOCK_TIME) {
                doneUnlocking = true;
                power = 0;
            }
            runIntake(power);
        } else {
            if (robot.shooter.getHopperPosition() != HopperModule.HopperPosition.LOWERED) {

            } else {
                holdRing = false;
            }

            double power = holdRing ? 0 : intakePower;

            runIntake(power);
        }
    }

    private void runIntake(double power) {
        intakeTop.setPower(power);
        intakeBottom.setPower(power);
    }

    private void intakeBlockerLogic() {
        intakeBlockerLogic(true);
        intakeBlockerLogic(false);
    }

    private void intakeBlockerLogic(boolean isLeft) {
        Point blockerLocation = blockerPosition(isLeft);

        // check if we're close to a wall
        for (int i = 0; i <= PERIMETER_VERTICES.length; i++) {
            Point vertex = PERIMETER_VERTICES[i];
            Point nextVertex = i == PERIMETER_VERTICES.length - 1 ? PERIMETER_VERTICES[0] : PERIMETER_VERTICES[i + 1];

            if (lineSegmentPointDistance(blockerLocation, vertex, nextVertex) < WALL_AVOID_DISTANCE) {
                setBlockerPosition(0, isLeft);
                return;
            }
        }

        switch (blockerPosition) {
            case BLOCKING:
                setBlockerPosition(0, isLeft);
                break;
            case OPEN:
                blockerFunnel(isLeft);
                break;
        }
    }

    private void blockerFunnel(boolean isLeft) {
        double headingToHighGoal = robot.shooter.relativeHeadingToTarget(robot.shooter.target.getAllianceHigh());

        double targetAngle = robotRelativeHeadingToBlockerHeading(headingToHighGoal, isLeft) + BLOCKER_FUNNEL_ANGLE;

        if (targetAngle > Math.toRadians(180)) {
            targetAngle = 0;
        }

        setBlockerPosition(targetAngle, isLeft);
    }

    private static final double LEFT_BLOCKER_SERVOPOS_SCALE = ((LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[0].y - LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[1].y)
            / (LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[0].x - LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[1].x));
    private static final double LEFT_BLOCKER_SERVOPOS_CONSTANT = LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[0].y - (LEFT_BLOCKER_SERVOPOS_SCALE * LEFT_BLOCKER_KNOWN_SERVO_POSITIONS[0].x);

    private static final double RIGHT_BLOCKER_SERVOPOS_SCALE = ((RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[0].y - RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[1].y)
            / (RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[0].x - RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[1].x));
    private static final double RIGHT_BLOCKER_SERVOPOS_CONSTANT = RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[0].y - (RIGHT_BLOCKER_SERVOPOS_SCALE * RIGHT_BLOCKER_KNOWN_SERVO_POSITIONS[0].x);

    private void setBlockerPosition(double angle, boolean isLeft) {
        if (isLeft) {
            double position = Range.clip((angle * LEFT_BLOCKER_SERVOPOS_SCALE) + LEFT_BLOCKER_SERVOPOS_CONSTANT, 0, 1);

            leftBlocker.setPosition(position);
        } else {
            double position = Range.clip((angle * RIGHT_BLOCKER_SERVOPOS_SCALE) + RIGHT_BLOCKER_SERVOPOS_CONSTANT, 0, 1);

            rightBlocker.setPosition(position);
        }
    }

    /**
     * Converts a heading relative to the robot to the blocker's heading system, where 0 is in front
     * of the intake and positive is outwards.
     *
     * @param heading A heading relative to the robot (the direction the robot is facing is 0)
     * @param isLeft  Whether to convert to the left blocker's system. If false, the right blocker.
     * @return The heading converted into the blocker's heading system.
     */
    private double robotRelativeHeadingToBlockerHeading(double heading, boolean isLeft) {
        if (isLeft) {
            return Math.toRadians(90) - heading;
        } else {
            return Math.toRadians(90) + heading;
        }
    }

    private Point blockerPosition(boolean isLeft) {
        return isLeft ?
                robot.drivetrain.positionOfRobotPart(LEFT_BLOCKER_POSITION)
                : robot.drivetrain.positionOfRobotPart(RIGHT_BLOCKER_POSITION);
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Intake power: " + intakePower);
        data.add("Blocker position: " + blockerPosition.toString());
        data.add("Done unlocking: " + doneUnlocking);
        data.add("Passes of ring: " + distanceSensorPasses);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }

    public void removeQueued() {
        this.distanceSensorPasses = 0;
    }

    public int getDistanceSensorPasses() {
        return this.distanceSensorPasses;
    }
}
