package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;

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

    // States
    public double flyWheelTargetSpeed;
    public double shooterFlapPosition;
    public boolean indexRing;

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

    @Override
    public void update() {
        // Ensure flywheel is up to speed, index and shoot if commanded to shoot.
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);

        shooterFlap.setPosition(shooterFlapPosition);

        updateIndexer();
    }

    private long indexTime = 0;

    /**
     * Logic to update the position of the indexer.
     */
    private void updateIndexer() {
        if (flyWheelTargetSpeed > 0) {
            long currentTime = SystemClock.elapsedRealtime();

            boolean indexerReturned = currentTime > indexTime + 1200;
            if (indexRing && indexerReturned) {
                if (upToSpeed()) {
                    indexerServo.setPosition(INDEX_PUSH_POSITION);
                    indexTime = currentTime;
                    indexRing = false;
                }
            } else if (indexRing) {
                indexRing = false;
            }

            boolean isDoneIndexing = currentTime > indexTime + 600;
            if (isDoneIndexing) {
                indexerServo.setPosition(INDEX_OPEN_POSITION);
            }
        }
    }

    /**
     * Sets states of the shooter module to shoot a ring. This includes ramping up the flywheel
     * and firing the ring when ready.
     *
     * @param flyWheelTargetSpeed The target speed of the flywheel.
     */
    public void setStatesToShoot(double flyWheelTargetSpeed) {
        this.flyWheelTargetSpeed = flyWheelTargetSpeed;
        indexRing = true;
    }

    /**
     * Sets states of the shooter module to shoot a ring. This includes ramping up the flywheel
     * and firing the ring when ready.
     *
     * @param flyWheelTargetSpeed The target speed of the flywheel.
     * @param shooterFlapPosition The position to move the shooter flap to.
     */
    public void setStatesToShoot(double flyWheelTargetSpeed, double shooterFlapPosition) {
        this.flyWheelTargetSpeed = flyWheelTargetSpeed;
        this.shooterFlapPosition = shooterFlapPosition;
        indexRing = true;
    }

    /**
     * Checks to see if the flywheel is up to speed.
     *
     * @return true if the flywheel speed is within the threshold of the target speed.
     */
    public boolean upToSpeed() {
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
        data.add("Flywheel speed: " + robot.shooterModule.flyWheel1.getVelocity());
        data.add("Flap angle: " + shooterFlapPosition);
        data.add("Will index: " + indexRing);

        return data;
    }

    @Override
    public void fileDump() {
        // TODO
    }

    public String getName() {
        return "ShooterModule";
    }
}
