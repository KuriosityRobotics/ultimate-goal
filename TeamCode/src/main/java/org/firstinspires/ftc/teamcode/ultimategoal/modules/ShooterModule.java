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

    private static final int FLYWHEEL_SPEED_THRESHOLD = 50;

    private static final double INDEX_OPEN_POSITION = .85;
    private static final double INDEX_PUSH_POSITION = .68;

    private static final int INDEXER_PUSHED_TIME_MS = 600;
    private static final int INDEXER_RETURNED_TIME_MS = 1200;

    private static final double HOPPER_UP_POSITION = 0.96;

    // States
    public double flyWheelTargetSpeed;
    public double shooterFlapPosition = 0.63;
    private boolean indexRing;

    // Motors
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo shooterFlap;

    private long indexTime = 0;

    public ShooterModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    @Override
    public void initModules() {
        flyWheel1 = (DcMotorEx) robot.getDcMotor("flyWheel1");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("flyWheel2");

        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterFlap = robot.getServo("shooterFlap");

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean initCycle() {
        flyWheel1.setVelocityPIDFCoefficients(8, 0.65, 0, 11.7);
        flyWheel2.setVelocityPIDFCoefficients(8, 0.65, 0, 11.7);
        return true; // No iterative init required
    }

    @Override
    public void update() {
        // Ensure flywheel is up to speed, index and shoot if commanded to shoot.
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);
        shooterFlap.setPosition(shooterFlapPosition);
    }

    public boolean isUpToSpeed() {
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
        data.add("PID coeefs: " + flyWheel1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        return data;
    }

    public String getName() {
        return "ShooterModule";
    }
}
