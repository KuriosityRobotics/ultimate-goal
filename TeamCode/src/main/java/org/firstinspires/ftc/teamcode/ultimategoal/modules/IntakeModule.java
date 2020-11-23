package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class IntakeModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public double intakePower = 0;

    // Actuators
    DcMotor intakeMotor;
    Servo leftIntakeLock;
    Servo rightIntakeLock;

    // Constants
    private static final double LEFT_LOCKED_POSITION = 0.13;
    private static final double LEFT_UNLOCKED_POSITION = 0;
    private static final double RIGHT_LOCKED_POSITION = 0.13;
    private static final double RIGHT_UNLOCKED_POSITION = 0;

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        intakeMotor = robot.getDcMotor("intakeMotor");

        leftIntakeLock = robot.getServo("leftIntakeLock");
        rightIntakeLock = robot.getServo("rightIntakeLock");

        setIntakeLocks(true);
    }

    public boolean initCycle() {
        return true;
    }

    long startTime = 0;
    boolean doneUnlocking = false;
    public void update() {
        if (!doneUnlocking) {
            if (startTime == 0) {
                startTime = robot.getCurrentTimeMilli();
                setIntakeLocks(false);
            } else if (robot.getCurrentTimeMilli() > startTime + 1000) {
                leftIntakeLock.getController().pwmDisable();
                doneUnlocking = true;
            }
        } else {
            if (robot.shooter.getHopperPosition() == HopperModule.HopperPosition.LOWERED) {
                intakeMotor.setPower(intakePower);
            } else {
                intakeMotor.setPower(0);
            }
        }
    }

    private void setIntakeLocks(boolean isLocked) {
        if (isLocked) {
            leftIntakeLock.setPosition(LEFT_LOCKED_POSITION);
            rightIntakeLock.setPosition(RIGHT_LOCKED_POSITION);
        } else {
            leftIntakeLock.setPosition(LEFT_UNLOCKED_POSITION);
            rightIntakeLock.setPosition(RIGHT_UNLOCKED_POSITION);
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Intake power: " + intakePower);
        data.add("Done unlocking: " + doneUnlocking);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
