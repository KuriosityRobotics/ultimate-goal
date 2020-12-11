package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class WobbleModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean isClawClamped = true;
    public WobbleArmPosition wobbleArmPosition = WobbleArmPosition.RAISED;

    private int wobbleEncoderTarget;

    public enum WobbleArmPosition {RAISED, WALL_DROP, LOWERED, AUTO_DROP}

    // Actuators
    DcMotorEx wobbleMotor;
    Servo wobbleClaw;

    // Constants
    private static final double CLAW_CLAMP_POSITION = 0.68;
    private static final double CLAW_OPEN_POSITION = 0.2;

    public static final int WOBBLE_RAISED_POSITION = 0;
    public static final int WOBBLE_LOWERED_POSITION = -620;
    public static final int WOBBLE_WALL_DROP_POSITION = -250;
    public static final int WOBBLE_AUTO_DROP_POSITION = -450;

    public static final int CLAW_CLOSE_MS = 750;
    public static final int CLAW_OPEN_MS = 750;

    public WobbleModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        wobbleMotor = (DcMotorEx) robot.getDcMotor("wobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleClaw = robot.getServo("wobbleClaw");

        wobbleClaw.setPosition(CLAW_CLAMP_POSITION);
    }

    public boolean initCycle() {
        return true;
    }

    boolean oldIsClawClamped = !isClawClamped;
    long clawTransitionTime = 0;

    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        if (oldIsClawClamped != isClawClamped) {
            if (isClawClamped) {
                wobbleClaw.setPosition(CLAW_CLAMP_POSITION);
            } else {
                wobbleClaw.setPosition(CLAW_OPEN_POSITION);
            }

            clawTransitionTime = currentTime;
            oldIsClawClamped = isClawClamped;
        }

        switch (wobbleArmPosition) {
            case RAISED:
                wobbleEncoderTarget = WOBBLE_RAISED_POSITION;
                break;
            case LOWERED:
                wobbleEncoderTarget = WOBBLE_LOWERED_POSITION;
                break;
            case WALL_DROP:
                wobbleEncoderTarget = WOBBLE_WALL_DROP_POSITION;
                break;
            case AUTO_DROP:
                wobbleEncoderTarget = WOBBLE_AUTO_DROP_POSITION;
                break;
        }

        double rawValue = Range.clip((wobbleMotor.getCurrentPosition() - wobbleEncoderTarget) / 100.0, -1.0, 1.0);

        wobbleMotor.setPower(rawValue * 0.5);
    }

    public void nextArmPosition() {
        switch (wobbleArmPosition) {
            case AUTO_DROP:
            case WALL_DROP:
                wobbleArmPosition = WobbleArmPosition.RAISED;
                break;
            case RAISED:
                wobbleArmPosition = WobbleArmPosition.LOWERED;
                break;
            case LOWERED:
                wobbleArmPosition = WobbleArmPosition.WALL_DROP;
                break;
        }
    }

    public boolean isClawAtPosition() {
        long currentTime = robot.getCurrentTimeMilli();

        if (isClawClamped) {
            return currentTime > clawTransitionTime + CLAW_CLOSE_MS && wasClawTransitionedToTarget();
        } else {
            return currentTime > clawTransitionTime + CLAW_OPEN_MS && wasClawTransitionedToTarget();
        }
    }

    public boolean wasClawTransitionedToTarget() {
        return oldIsClawClamped == isClawClamped;
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Wobble target position: " + wobbleArmPosition);
        data.add("Wobble encoder target position: " + wobbleEncoderTarget);
        data.add("Wobble current position: " + wobbleMotor.getCurrentPosition());
        data.add("Is claw clamped: " + isClawClamped);
        data.add("Is claw at position: " + isClawAtPosition());
        return data;
    }

    public String getName() {
        return "WobbleModule";
    }
}

