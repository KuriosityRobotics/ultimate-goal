package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.enhancedhardware.EnhancedServo;

import java.util.ArrayList;

public class ShooterModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public double flyWheelTargetSpeed;
    public double shooterFlapAngle; // Radians, relative to horizon.
    public boolean indexRing;

    // Motors
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private EnhancedServo shooterFlap;
    private EnhancedServo indexerServo;

    public ShooterModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    @Override
    public void init() {
        // TODO
        flyWheel1 = (DcMotorEx) robot.getDcMotor("yLeft");
        flyWheel2 = (DcMotorEx) robot.getDcMotor("yRight");
        shooterFlap = new EnhancedServo(robot.getServo("shooterFlap"), 0, 180); // In degrees
        indexerServo = new EnhancedServo(robot.getServo("indexerServo"), 0, 180);

        flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private long indexTime  = 0;
    @Override
    public void update() {
        // TODO
        // Ensure flywheel is up to speed, index and shoot if commanded to shoot.
        flyWheel1.setVelocity(flyWheelTargetSpeed);
        flyWheel2.setVelocity(flyWheelTargetSpeed);
        shooterFlap.setAngle(shooterFlapAngle);

        long currentTime = SystemClock.elapsedRealtime();
        boolean isDoneIndexing = currentTime > indexTime + 1000;
        boolean indexerReturned = currentTime > indexTime + 5000;
        if (indexRing && indexerReturned) {
            indexerServo.setAngle(180);
            indexTime = currentTime;
            indexRing = false;
        } else if (indexRing) {
            indexRing = false;
        }
        if (isDoneIndexing) {
            indexerServo.setAngle(0);
        }
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        // TODO
        ArrayList<String> data = new ArrayList<>();
        data.add("Flywheel speed: " + flyWheelTargetSpeed);
        data.add("Flap angle: " + shooterFlapAngle);
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
