package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class WobbleModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean isClawClamped;
    public int wobbleTargetPosition;

    // Actuators
    DcMotor wobbleMotor;
    Servo wobbleClaw;

    // Constants
    private static final double CLAW_CLAMP_POSITION = 0.67;
    private static final double CLAW_OPEN_POSITION = 0.2;

    public static final int WOBBLE_RAISED_POSITION = 0; // TODO tune
    public static final int WOBBLE_LOWERED_POSITION = 100;

    public WobbleModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        wobbleMotor = robot.getDcMotor("wobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleClaw = robot.getServo("wobbleClaw");
    }

    public boolean initCycle() {
        return true;
    }

    public void update() {
        if (isClawClamped) {
            wobbleClaw.setPosition(CLAW_CLAMP_POSITION);
        } else {
            wobbleClaw.setPosition(CLAW_OPEN_POSITION);
        }

        wobbleMotor.setTargetPosition(wobbleTargetPosition);
    }

    public void setWobbleArmPosition(boolean lowerWobble) {
        if (lowerWobble) {
            wobbleTargetPosition = WOBBLE_LOWERED_POSITION;
        } else {
            wobbleTargetPosition = WOBBLE_RAISED_POSITION;
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Wobble target position: " + wobbleTargetPosition);
        data.add("Is claw clamped: " + isClawClamped);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}

