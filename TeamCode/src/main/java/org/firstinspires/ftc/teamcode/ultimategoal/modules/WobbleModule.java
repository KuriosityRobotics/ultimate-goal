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
    public ClawPosition clawPosition = ClawPosition.OPEN;
    public double wobblePower;

    public enum ClawPosition {CLAMP, OPEN}

    // Actuators
    DcMotor wobbleMotor;
    Servo wobbleClaw;

    // Constants
    private static final double CLAW_CLAMP_POSITION = 0.67;
    private static final double CLAW_OPEN_POSITION = 0.2;

    public WobbleModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        wobbleMotor = robot.getDcMotor("wobbleMotor");

        wobbleClaw = robot.getServo("wobbleClaw");
    }

    public boolean initCycle() {
        return true;
    }

    long startTime = 0;

    private ClawPosition oldHopperPosition = ClawPosition.OPEN;

    public void update() {
        if (clawPosition != oldHopperPosition) {
            if (clawPosition == ClawPosition.CLAMP) {
                wobbleClaw.setPosition(CLAW_CLAMP_POSITION);
            } else {
                wobbleClaw.setPosition(CLAW_OPEN_POSITION);
            }

            oldHopperPosition = clawPosition;
        }
        wobbleMotor.setPower(wobblePower);
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}

