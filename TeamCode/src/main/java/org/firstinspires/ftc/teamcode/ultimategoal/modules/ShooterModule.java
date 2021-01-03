package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class ShooterModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    private static final int FLYWHEEL_SPEED_THRESHOLD = 50;
    private static final double FLYWHEEL_P = 0.00008;
    private static final double FLYWHEEL_D = 0.04;

    // States
    public double flyWheelTargetSpeed;
    public double shooterFlapPosition = 0.63;

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
    long startTime = 0;
    @Override
    public void initModules() {
        flyWheel1 = (DcMotorEx) robot.getDcMotor("flyWheel1");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("flyWheel2");

        flyWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterFlap = robot.getServo("shooterFlap");

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyWheel1.setVelocityPIDFCoefficients(8, 0.6, 0, 11.7);
        flyWheel2.setVelocityPIDFCoefficients(8, 0.6, 0, 11.7);
    }

    @Override
    public void update() {
        // Ensure flywheel is up to speed, index and shoot if commanded to shoot.
        setFlywheelMotors();

        shooterFlap.setPosition(Range.clip(shooterFlapPosition, .65, .68));
    }

    double shooterPower = 1;
    double oldFlywheelTarget = 0;

    double lastError = 0;
    long lastTime = 0;

    private void setFlywheelMotors() {
        if(startTime == 0){
            startTime = SystemClock.elapsedRealtime();
        }
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);
//
//        if (oldFlywheelTarget != flyWheelTargetSpeed) {
//            shooterPower = flyWheelTargetSpeed / 1550;
//            oldFlywheelTarget = flyWheelTargetSpeed;
//        }
//
//        double flywheelPower = 0;
//        double currentFlywheelVelocity = flyWheel1.getVelocity();
//
//        if (flyWheelTargetSpeed == 0) {
//            flywheelPower = 0;
//        } else if (currentFlywheelVelocity < flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD) {
//            flywheelPower = 1;
//        } else {
//            long currentTime = robot.getCurrentTimeMilli();
//
//            double diffVelocity = flyWheelTargetSpeed - currentFlywheelVelocity;
//
//            double changeError = (diffVelocity - lastError) / (currentTime - lastTime);
//
//            double increment = (diffVelocity * FLYWHEEL_P) + (changeError * FLYWHEEL_D);
//
//            shooterPower = Range.clip(shooterPower + increment, -1, 1);
//
//            flywheelPower = shooterPower;
//
//            lastTime = currentTime;
//            lastError = diffVelocity;
//        }
//
//        flyWheel1.setPower(flywheelPower);
//        flyWheel2.setPower(flywheelPower);
        if(isUpToSpeed()){
            Log.d("NYOOM", Long.toString(SystemClock.elapsedRealtime()-startTime));
        }
    }

    public boolean isUpToSpeed() {
        return flyWheel1.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD
                && flyWheel2.getVelocity() > flyWheelTargetSpeed - FLYWHEEL_SPEED_THRESHOLD;
    }

    public double getFlyWheelVelocity(){
        return flyWheel1.getVelocity();
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
        data.add("Flywheel power from PID: " + shooterPower);
        data.add("Flap angle: " + shooterFlapPosition);
        return data;
    }

    public String getName() {
        return "ShooterModule";
    }
}
