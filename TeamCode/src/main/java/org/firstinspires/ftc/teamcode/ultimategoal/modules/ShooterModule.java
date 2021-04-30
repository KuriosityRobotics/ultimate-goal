package org.firstinspires.ftc.teamcode.ultimategoal.modules;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;

@Config
public class ShooterModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    public static double turretP = 15;
    public static double turretI = 3;
    public static double turretD = 12;
    public static double turretF = 3;

    public static double flywheelP = 45;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 15.5;

    public static double feedForwardVelocityTurret = 8;

    // States
    private double targetTurretAngle;
    public double flyWheelTargetSpeed;

    public boolean indexRing;
    public double shooterFlapPosition = FLAP_LOWER_LIMIT;

    public boolean manualTurret = false;
    public double manualPower = 0;

    public boolean limitAngle = false;
    public double lowerAngleLimit = -Math.PI;
    public double upperAngleLimit = Math.PI;

    // Data
    private double currentTurretAngle;
    private IndexerPosition indexerPosition;

    // Motors
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private DcMotorEx turretMotor;

    // Servos
    private Servo indexerServo;
    private Servo shooterFlap;

    // Helpers
    private boolean lastIterationFinishedIndex;
    private long indexTime;

    private double lastTurretPosition;
    private double turretVelocity;
    private double turretPower;

    // Constants
    private static final double TURRET_MINIMUM_ANGLE = Math.toRadians(-215);
    private static final double TURRET_MAXIMUM_ANGLE = Math.toRadians(270);

    private static final double TURRET_ENCODER_TO_ANGLE = 227.2959951557;
    private static final int FLYWHEEL_SPEED_THRESHOLD = 25;

    private static final double FLAP_STORE_POSITION = 0.0873856;
    private static final double FLAP_LOWER_LIMIT = 0.1766;
    private static final double FLAP_UPPER_LIMIT = 0.29255;

    private static final double INDEXER_PUSHED_POSITION = 0.4; //.45
    private static final double INDEXER_RETRACTED_POSITION = 0.72;

    private static final int INDEXER_PUSHED_TIME_MS = 160; // since start of index
    public static final int INDEXER_RETURNED_TIME_MS = 400; // since start of index

    public enum IndexerPosition {RETRACTED, PUSHED}

    public ShooterModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;

        targetTurretAngle = 0;

        flyWheelTargetSpeed = 0;

        indexRing = false;
        indexerPosition = IndexerPosition.RETRACTED;
        indexTime = 0;

        currentTurretAngle = 0;
    }

    @Override
    public void initModules() {
        initFlywheels();

        turretMotor = (DcMotorEx) robot.getDcMotor("turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(turretP, turretI, turretD, turretF));

        indexerServo = robot.getServo("indexer");

        shooterFlap = robot.getServo("shooterFlap");
    }

    private void initFlywheels() {
        flyWheel1 = (DcMotorEx) robot.getDcMotor("flyWheel1");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("flyWheel2");

        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyWheel1.setVelocityPIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        flyWheel2.setVelocityPIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
    }

    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        flyWheel1.setVelocityPIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        flyWheel2.setVelocityPIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);

        turretMotor.setVelocityPIDFCoefficients(turretP, turretI, turretD, turretF);

        calculateTurretPosition();

        if (!manualTurret) {
            turnTurretToTarget();
        } else {
            turnTurret(manualPower);
        }

        shooterFlap.setPosition(Range.clip(shooterFlapPosition, FLAP_LOWER_LIMIT, FLAP_UPPER_LIMIT));

        setFlywheelMotors();

        indexerLogic(currentTime);
    }

    long oldUpdateTime;

    private void calculateTurretPosition() {
        long currentUpdateTime = robot.getCurrentTimeMilli();

        this.currentTurretAngle = turretMotor.getCurrentPosition() / TURRET_ENCODER_TO_ANGLE;

        turretVelocity = (currentTurretAngle - lastTurretPosition) / (currentUpdateTime - oldUpdateTime) * 1000;

        lastTurretPosition = currentTurretAngle;

        oldUpdateTime = currentUpdateTime;
    }

    private void turnTurretToTarget() {
        turretMotor.setTargetPosition((int) ((targetTurretAngle * TURRET_ENCODER_TO_ANGLE) + turretVelocity * feedForwardVelocityTurret));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (turretMotor.isBusy()) {
            turretMotor.setPower(1);
        } else {
            turretMotor.setPower(0);
        }
    }

    private void turnTurret(double power) {
        if (power == 0) {
            turretMotor.setPower(0);

            return;
        }

        power = Range.clip(power, -1, 1);

        turretMotor.setPower(power);
    }

    private void setFlywheelMotors() {
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);

        if (flywheelsUpToSpeed()) {
            robot.setLedColor(0, 200, 200);
        } else {
            robot.setLedColor(25, 25, 25);
        }
    }

    private void indexerLogic(long currentTime) {
        boolean finishedIndexing = currentTime >= indexTime + INDEXER_RETURNED_TIME_MS;

        if (finishedIndexing && !lastIterationFinishedIndex) {
            indexRing = false;
        }

        if (currentTime < indexTime + INDEXER_PUSHED_TIME_MS) {
            indexerPosition = IndexerPosition.PUSHED;
        } else if (currentTime < indexTime + INDEXER_RETURNED_TIME_MS) {
            indexerPosition = IndexerPosition.RETRACTED;
        } else if (indexRing) {
            indexTime = currentTime;

            indexerPosition = IndexerPosition.PUSHED;
        } else {
            indexerPosition = IndexerPosition.RETRACTED;
        }

        setIndexerPosition();

        lastIterationFinishedIndex = finishedIndexing;
    }

    private void setIndexerPosition() {
        switch (indexerPosition) {
            case PUSHED:
                indexerServo.setPosition(INDEXER_PUSHED_POSITION);
                break;
            case RETRACTED:
                indexerServo.setPosition(INDEXER_RETRACTED_POSITION);
                break;
        }
    }

    public boolean flywheelsUpToSpeed() {
//        Log.v("shootermod", ""+flyWheel1.getVelocity());
//        Log.v("shootermod", ""+flyWheel2.getVelocity());

        return (flyWheel1.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD
                || flyWheel2.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD)
                && flyWheelTargetSpeed > 0;
    }

    /**
     * Request for the turret to turn to face a given angle, relative to the robot.
     *
     * @param angle
     */
    public void setTargetTurretAngle(double angle) {
        double wrapped = angleWrap(angle);

        if (limitAngle) {
            wrapped = new android.util.Range<>(lowerAngleLimit, upperAngleLimit).clamp(wrapped);
        }

        double target = currentTurretAngle + angleWrap(wrapped - currentTurretAngle);

        while (target > TURRET_MAXIMUM_ANGLE) {
            target -= 2 * Math.PI;
        }

        while (target < TURRET_MINIMUM_ANGLE) {
            target += 2 * Math.PI;
        }

        targetTurretAngle = target;
    }

    public double getCurrentTurretAngle() {
        return currentTurretAngle;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }

    /**
     * Get the error of the turret's angle.
     *
     * @return The target turret angle minus the current turret angle, in radians.
     */
    public double getTurretError() {
        return targetTurretAngle - currentTurretAngle;
    }

    public boolean isFinishedIndexing() {
        return isIndexerReturned() && !indexRing;
    }

    public boolean isIndexerReturned() {
        return robot.getCurrentTimeMilli() > indexTime + INDEXER_RETURNED_TIME_MS;
    }

    public IndexerPosition getIndexerPosition() {
        return indexerPosition;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("turretPos", Math.toDegrees(currentTurretAngle));
        data.put("turretTargetPos", Math.toDegrees(targetTurretAngle));
        data.put("turret Power: ", turretPower);
        data.put("flywheel 1 velocity: ", flyWheel1.getVelocity());
        data.put("flywheel 2 velocity: ", flyWheel2.getVelocity());
        data.put("flywheel target velocity : ", flyWheelTargetSpeed);
        data.put("flywheel 1 PID coeff ", flyWheel1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        data.put("turret PID coeff ", turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));


        return data;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Target speed: " + flyWheelTargetSpeed);
        data.add("Flywheel1 speed: " + flyWheel1.getVelocity() + ", flywheel2 speed: " + flyWheel2.getVelocity());
        data.add("isUpToSpeed: " + flywheelsUpToSpeed());
        data.add("--");
        data.add("manualTurret: " + manualTurret + " manualPower: " + manualPower);
        data.add("Target turret angle: " + Math.toDegrees(targetTurretAngle) + ", Target turret encoders: " + turretMotor.getTargetPosition());
        data.add("Current turret angle: " + Math.toDegrees(currentTurretAngle));
        data.add("Flap angle: " + shooterFlapPosition);
        return data;
    }

    public String getName() {
        return "ShooterModule";
    }
}
