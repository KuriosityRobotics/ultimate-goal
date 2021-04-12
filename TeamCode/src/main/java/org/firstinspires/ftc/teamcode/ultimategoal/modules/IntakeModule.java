package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.wrappers.AnalogDistance;

import java.util.ArrayList;

public class IntakeModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public double intakePower = 0;

    // Actuators
    DcMotor intakeTop;
    DcMotor intakeBottom;


    // Sensors
    AnalogInput intakeDistance;

    // Helpers
    boolean holdRing = false;

    // Constants
    private static final double LOCKS_LOCKED_POSITION = 0.289; // position for left, right is 1 - left
    private static final double LOCKS_UNLOCKED_POSITION = 0.402;

    public IntakeModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
        this.timeLastPass = robot.getCurrentTimeMilli();

    }

    public void initModules() {
        intakeDistance = robot.hardwareMap.get(AnalogInput.class, "distance");

        intakeTop = robot.getDcMotor("intakeTop");
        intakeBottom = robot.getDcMotor("intakeBottom");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    long startTime = 0;
    boolean doneUnlocking = false;

    private double timeLastPass;


    boolean seenRing = false;
    int distanceSensorPasses = 0;

    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        double voltage = intakeDistance.getVoltage();
        if (Math.abs(2.2 - voltage) < Math.abs(1.1 - voltage)) {
            if (!seenRing)
                seenRing = true; // we need to track when the sensor starts seeing it, and when it finishes

        } else if (seenRing) {
            if (robot.intakeModule.intakeBottom.getPower() > 0) // outtaking or intaking ?
                distanceSensorPasses = distanceSensorPasses + 1;
            else
                distanceSensorPasses = distanceSensorPasses - 1;
            seenRing = false;

        }
        if (currentTime - timeLastPass > 200 && (distanceSensorPasses & 1) == 1 && seenRing) { // emergency in case it somehow gets out of sync:  timeout after two seconds & round up
            distanceSensorPasses++;
        }
        timeLastPass = currentTime;



        if (!doneUnlocking) {
            if (startTime == 0) {
                startTime = robot.getCurrentTimeMilli();
            } else if (robot.getCurrentTimeMilli() > startTime + 750) {
                doneUnlocking = true;
            }
        } else {

            if (robot.shooter.getHopperPosition() != HopperModule.HopperPosition.LOWERED) {

            } else {
                holdRing = false;
            }

            double power = holdRing ? 0 : intakePower;

            intakeTop.setPower(power);
            intakeBottom.setPower(power);
        }
    }




    public boolean isOn() {
        return isOn;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Intake power: " + intakePower);
        data.add("Done unlocking: " + doneUnlocking);
        data.add("Passes of ring: " + distanceSensorPasses);
        return data;
    }

    public String getName() {
        return "IntakeModule";
    }

    public void removeQueued() {
        this.distanceSensorPasses = 0;
    }

    public int getDistanceSensorPasses() {
        return this.distanceSensorPasses;
    }
}
