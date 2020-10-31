package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class ShooterModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    private static final int FLYWHEEL_SPEED_THRESHOLD = 90;

    private static final double INDEX_OPEN_POSITION = 0.75;
    private static final double INDEX_PUSH_POSITION = 0.15;

    private static final int INDEXER_PUSHED_TIME_MS = 600;
    private static final int INDEXER_RETURNED_TIME_MS = 1200;

    // States
    public double flyWheelTargetSpeed;
    public double shooterFlapPosition = 0.71;
    private boolean indexRing;

    // Motors
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo shooterFlap;
    private Servo indexerServo;

    public ShooterModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    @Override
    public void init() {
        flyWheel1 = (DcMotorEx) robot.getDcMotor("flyWheel1");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("flyWheel2");

        flyWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterFlap = robot.getServo("shooterFlap"); // In degrees
        indexerServo = robot.getServo("indexerServo");

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private long indexTime = 0;

    @Override
    public void update() {
        // Ensure flywheel is up to speed, index and shoot if commanded to shoot.
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);

        shooterFlap.setPosition(shooterFlapPosition);

        long currentTime = robot.getCurrentTimeMilli();

        boolean indexerReturned = currentTime > indexTime + INDEXER_RETURNED_TIME_MS;
        if (indexRing && indexerReturned && upToSpeed()) {
            indexerServo.setPosition(INDEX_PUSH_POSITION);
            indexTime = currentTime;
            indexRing = false;
        }

        boolean isDoneIndexing = currentTime > indexTime + INDEXER_PUSHED_TIME_MS;
        if (isDoneIndexing) {
            indexerServo.setPosition(INDEX_OPEN_POSITION);
        }
    }

    /**
     * Attempt to index a ring. If the indexer is currently indexing, nothing will happen. Whether
     * or not the command was successfully executed is returned.
     *
     * @return Whether or not the index command will be processed.
     */
    public boolean indexRing() {
        if (robot.getCurrentTimeMilli() > indexTime + INDEXER_RETURNED_TIME_MS) {
            indexRing = true;
            return true;
        } else {
            return false;
        }
    }

    private boolean upToSpeed() {
        return Math.abs(flyWheel1.getVelocity() - flyWheelTargetSpeed) < FLYWHEEL_SPEED_THRESHOLD
                && Math.abs(flyWheel2.getVelocity() - flyWheelTargetSpeed) < FLYWHEEL_SPEED_THRESHOLD;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Flywheel speed: " + flyWheel1.getVelocity());
        data.add("Flap angle: " + shooterFlapPosition);
        data.add("Will index: " + indexRing);

        return data;
    }

    public String getName() {
        return "ShooterModule";
    }
}
