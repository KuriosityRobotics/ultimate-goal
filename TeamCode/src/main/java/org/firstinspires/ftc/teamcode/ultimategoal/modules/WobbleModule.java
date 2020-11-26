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
    public WobbleArmPosition wobbleArmPosition;

    private double wobbleEncoderTarget;

    public enum WobbleArmPosition {RAISED, DEPOSIT, LOWERED}

    // Actuators
    DcMotor wobbleMotor;
    Servo wobbleClaw;

    // Constants
    private static final double CLAW_CLAMP_POSITION = 0.67;
    private static final double CLAW_OPEN_POSITION = 0.2;

    public static final int WOBBLE_RAISED_POSITION = 0;
    public static final int WOBBLE_LOWERED_POSITION = -620;
    public static final int WOBBLE_WALL_DROP_POSITION = -300;

    public WobbleModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        wobbleMotor = robot.getDcMotor("wobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        switch (wobbleArmPosition) {
            case RAISED:
                wobbleEncoderTarget = WOBBLE_RAISED_POSITION;
                break;
            case LOWERED:
                wobbleEncoderTarget = WOBBLE_LOWERED_POSITION;
                break;
            case DEPOSIT:
                wobbleEncoderTarget = WOBBLE_WALL_DROP_POSITION;
                break;
        }

        wobbleMotor.setPower((wobbleMotor.getCurrentPosition() - ((int) wobbleEncoderTarget)) / 100);
    }

    public void setWobbleArmPosition(WobbleArmPosition position) {
        wobbleArmPosition = position;
    }

    public void nextArmPosition() {
        switch (wobbleArmPosition) {
            case RAISED:
                wobbleArmPosition = WobbleArmPosition.DEPOSIT;
                break;
            case LOWERED:
                wobbleArmPosition = WobbleArmPosition.RAISED;
                break;
            case DEPOSIT:
                wobbleArmPosition = WobbleArmPosition.LOWERED;
                break;
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Wobble target position: " + wobbleArmPosition);
        data.add("Wobble current position: " + wobbleMotor.getCurrentPosition());
        data.add("Is claw clamped: " + isClawClamped);
        return data;
    }

    public String getName() {
        return "WobbleModule";
    }
}

