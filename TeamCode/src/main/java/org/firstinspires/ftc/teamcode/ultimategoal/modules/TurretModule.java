package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;

public class TurretModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public double targetTurretAngle;

    public double flyWheelTargetSpeed;

    public boolean indexRing;

    public double shooterFlapPosition = 0.63;

    // Data
    private double currentTurretAngle;

    // Motors
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;

    // Encoders
    private DcMotor turretEncoder;

    // Servos
    private CRServo turretServo;
    private Servo indexerServo;
    private Servo shooterFlap;

    // Helpers
    long indexTime;

    // Constants
    private static final double TURRET_ENCODER_TO_ANGLE = 0; // todo tune
    private static final int FLYWHEEL_SPEED_THRESHOLD = 50;

    private static final double TURRET_ANGLE_THRESHOLD = Math.toRadians(0.5);

    private static final double INDEXER_PUSHED_POSITION = 0.45;
    private static final double INDEXER_RETRACTED_POSITION = 0.72;

    private static final int INDEXER_PUSHED_TIME_MS = 150; // since start of index
    private static final int INDEXER_RETURNED_TIME_MS = 300; // since start of index

    public TurretModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;

        targetTurretAngle = 0;

        flyWheelTargetSpeed = 0;

        indexRing = false;
        indexTime = 0;

        currentTurretAngle = 0;
    }

    @Override
    public void initModules() {
        flyWheel1 = (DcMotorEx) robot.getDcMotor("flyWheel1");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("flyWheel2");

        flyWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyWheel1.setVelocityPIDFCoefficients(8, 0.6, 0, 11.7);
        flyWheel2.setVelocityPIDFCoefficients(8, 0.6, 0, 11.7);

        turretEncoder = robot.getDcMotor("intakeTop");

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretServo = robot.hardwareMap.crservo.get("turretServo");

        indexerServo = robot.getServo("indexerServo");

        shooterFlap = robot.getServo("shooterFlap");
    }

    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        turretLogic();

        setFlywheelMotors();

        indexerLogic(currentTime);

        shooterFlap.setPosition(Range.clip(shooterFlapPosition, 0.598, 0.73));
    }

    private void turretLogic() {
        this.currentTurretAngle = angleWrap(turretEncoder.getCurrentPosition() * TURRET_ENCODER_TO_ANGLE);

        this.targetTurretAngle = angleWrap(targetTurretAngle);

        if (Math.abs(currentTurretAngle - targetTurretAngle) < TURRET_ANGLE_THRESHOLD) {
            turretServo.setPower(0);
        } else if (targetTurretAngle > currentTurretAngle) {
            turretServo.setDirection(DcMotorSimple.Direction.FORWARD);
            turretServo.setPower(1);
        } else {
            turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
            turretServo.setPower(1);
        }
    }

    private void setFlywheelMotors() {
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);

        if (flywheelsUpToSpeed() && flyWheelTargetSpeed > 0) {
            robot.setLedColor(0, 200, 200);
        } else {
            robot.setLedColor(25, 25, 25);
        }
    }

    private void indexerLogic(long currentTime) {
        if (currentTime < indexTime + INDEXER_PUSHED_TIME_MS) {
            indexerServo.setPosition(INDEXER_PUSHED_POSITION);
        } else if (currentTime < indexTime + INDEXER_RETURNED_TIME_MS) {
            indexerServo.setPosition(INDEXER_RETRACTED_POSITION);
        } else if (indexRing) {
            indexerServo.setPosition(INDEXER_PUSHED_POSITION);

            indexRing = false;
            indexTime = currentTime;
        } else {
            indexerServo.setPosition(INDEXER_RETRACTED_POSITION);
        }
    }

    public boolean flywheelsUpToSpeed() {
        return flyWheel1.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD
                && flyWheel2.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD;
    }

    public double getCurrentTurretAngle() {
        return currentTurretAngle;
    }

    public boolean isFinishedIndexing() {
        return robot.getCurrentTimeMilli() > indexTime + INDEXER_PUSHED_TIME_MS;
    }

    public boolean isIndexerReturned() {
        return robot.getCurrentTimeMilli() > indexTime + INDEXER_RETURNED_TIME_MS;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Target speed: " + flyWheelTargetSpeed);
        data.add("Flywheel1 speed: " + flyWheel1.getVelocity());
        data.add("Flywheel2 speed: " + flyWheel2.getVelocity());
        data.add("Flap angle: " + shooterFlapPosition);
        data.add("isUpToSpeed: " + flywheelsUpToSpeed());
        return data;
    }

    public String getName() {
        return "ShooterModule";
    }
}
