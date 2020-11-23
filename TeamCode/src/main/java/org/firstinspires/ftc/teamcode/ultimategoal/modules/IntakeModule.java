package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class IntakeModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public double intakePower;

    // Actuators
    DcMotor intakeMotor;

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    public void initModules() {
        intakeMotor = robot.getDcMotor("intakeMotor");
    }

    public boolean initCycle() {
        return true;
    }

    public void update() {
        if (robot.shooter.getHopperPosition() == HopperModule.HopperPosition.LOWERED) {
            intakeMotor.setPower(intakePower);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Intake power: " + intakePower);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }
}
