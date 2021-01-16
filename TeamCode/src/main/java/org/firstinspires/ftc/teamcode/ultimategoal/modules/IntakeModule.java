package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    DcMotor intakeTop;
    DcMotor intakeBottom;

    Servo leftIntakeLock;
    Servo rightIntakeLock;

    // Constants
    private static final double LOCKS_LOCKED_POSITION = 0.289; // position for left, right is 1 - left
    private static final double LOCKS_UNLOCKED_POSITION = 0.402;

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        intakeTop = robot.getDcMotor("intakeTop");
        intakeBottom = robot.getDcMotor("intakeBottom");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeLock = robot.getServo("leftIntakeLock");
        rightIntakeLock = robot.getServo("rightIntakeLock");

        setIntakeLocks(true);
    }

    long startTime = 0;
    boolean doneUnlocking = false;

    public void update() {
        if (!doneUnlocking) {
            if (startTime == 0) {
                startTime = robot.getCurrentTimeMilli();
                setIntakeLocks(false);
            } else if (robot.getCurrentTimeMilli() > startTime + 750) {
                doneUnlocking = true;
            }
        } else {
            double power = robot.shooter.getHopperPosition() == HopperModule.HopperPosition.LOWERED ? intakePower : 0;

            intakeTop.setPower(power);
            intakeBottom.setPower(power);
        }
    }

    private void setIntakeLocks(boolean isLocked) {
        if (isLocked) {
            setIntakeLocksPosition(LOCKS_LOCKED_POSITION);
        } else {
            setIntakeLocksPosition(LOCKS_UNLOCKED_POSITION);
        }
    }

    private void setIntakeLocksPosition(double position) {
        leftIntakeLock.setPosition(position);
        rightIntakeLock.setPosition(1 - position);
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
