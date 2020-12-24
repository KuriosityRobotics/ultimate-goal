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

    private double wobbleRightTargetPosition;
    private double wobbleLeftTargetPosition;

    public enum WobbleArmPosition {RAISED, WALL_DROP, LOWERED, AUTO_DROP}

    // Actuators
    Servo wobbleLeft;
    Servo wobbleRight;

    Servo wobbleClaw;

    // Constants
    private static final double CLAW_CLAMP_POSITION = 0.68;
    private static final double CLAW_OPEN_POSITION = 0.2;

    public static final double WOBBLE_RAISED_LEFT_POSITON = 0.225;
    public static final double WOBBLE_RAISED_RIGHT_POSITON = 0.775;

    public static final double WOBBLE_LOWERED_LEFT_POSITON = 0.86;
    public static final double WOBBLE_LOWERED_RIGHT_POSITON = 0.14;

    public static final double WOBBLE_WALLDROP_LEFT_POSITON = 0.55;
    public static final double WOBBLE_WALLDROP_RIGHT_POSITON = 0.45;

    public static final double WOBBLE_AUTODROP_LEFT_POSITON = 0.7;
    public static final double WOBBLE_AUTODROP_RIGHT_POSITON = 0.3;

    public static final int CLAW_CLOSE_MS = 750;
    public static final int CLAW_OPEN_MS = 750;

    public WobbleModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        wobbleLeft = robot.getServo("wobbleLeft");
        wobbleRight = robot.getServo("wobbleRight");

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
                wobbleLeftTargetPosition = WOBBLE_RAISED_LEFT_POSITON;
                wobbleRightTargetPosition = WOBBLE_RAISED_RIGHT_POSITON;
                break;
            case LOWERED:
                wobbleLeftTargetPosition = WOBBLE_LOWERED_LEFT_POSITON;
                wobbleRightTargetPosition = WOBBLE_LOWERED_RIGHT_POSITON;
                break;
            case WALL_DROP:
                wobbleLeftTargetPosition = WOBBLE_WALLDROP_LEFT_POSITON;
                wobbleRightTargetPosition = WOBBLE_WALLDROP_RIGHT_POSITON;
                break;
            case AUTO_DROP:
                wobbleLeftTargetPosition = WOBBLE_AUTODROP_LEFT_POSITON;
                wobbleRightTargetPosition = WOBBLE_AUTODROP_RIGHT_POSITON;
                break;
        }

        wobbleLeft.setPosition(wobbleLeftTargetPosition);
        wobbleRight.setPosition(wobbleRightTargetPosition);
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
        data.add("Is claw clamped: " + isClawClamped);
        data.add("Is claw at position: " + isClawAtPosition());
        return data;
    }

    public String getName() {
        return "WobbleModule";
    }
}

