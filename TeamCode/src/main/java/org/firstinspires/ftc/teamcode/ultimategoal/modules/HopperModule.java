package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class HopperModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean deliverRings;
    boolean wasAtTurret = false;

    // Data
    private int ringsInHopper;

    // Servos
    private Servo hopperLinkage;

    // Helpers
    private long deliveryStartTime = 0;

    AnalogInput ringCounterSensor;

    // Constants
    private static final double LINKAGE_LOWERED_POSITION = 0;
    private static final double LINKAGE_RAISED_POSITION = 0.36;

    private static final int RAISE_TIME_MS = 900; // from lowered to apex
    private static final int RAISE_TRANSITIONING_TIME_MS = RAISE_TIME_MS / 2; // from lowered to interfering with shooter
    private static final int LOWER_TIME_MS = 500; // from apex to lowered
    private static final int LOWER_CLEAR_SHOOTER_TIME_MS = LOWER_TIME_MS / 2; // from apex to no longer interfering with shooter

    // Hopper position enum
    public enum HopperPosition {LOWERED, TRANSITIONING, AT_TURRET}

    public HopperModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        deliverRings = false;
        deliveryStartTime = 0;
        ringsInHopper = 0;


    }

    @Override
    public void initModules() {
        hopperLinkage = robot.getServo("hopper");

        hopperLinkage.setPosition(LINKAGE_LOWERED_POSITION);
    }


    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();
        this.ringsInHopper = robot.intakeModule.getDistanceSensorPasses() / 2;


        HopperPosition currentHopperPosition = calculateHopperPosition(this.deliveryStartTime, currentTime);
// do the thing


        // Determine target hopper position
        boolean raiseHopper;
        if (currentHopperPosition == HopperPosition.LOWERED && this.deliverRings) {
            raiseHopper = true;

            this.deliveryStartTime = currentTime;
            deliverRings = false;
        } else {
            raiseHopper = currentTime < this.deliveryStartTime + RAISE_TIME_MS;
        }

        // Move hopper
        if (raiseHopper) {
            hopperLinkage.setPosition(LINKAGE_RAISED_POSITION);
            robot.shooter.turretModule.currentRingsInTurret += this.ringsInHopper;
            robot.intakeModule.removeQueued();
        } else {
            hopperLinkage.setPosition(LINKAGE_LOWERED_POSITION);
        }
    }

    private HopperPosition calculateHopperPosition(long deliveryStartTime, long currentTime) {
        if (currentTime < deliveryStartTime + RAISE_TRANSITIONING_TIME_MS) {
            return HopperPosition.TRANSITIONING;
        } else if (currentTime < deliveryStartTime + RAISE_TIME_MS + LOWER_CLEAR_SHOOTER_TIME_MS) {
            wasAtTurret = true;
            return HopperPosition.AT_TURRET;
        } else if (currentTime < deliveryStartTime + RAISE_TIME_MS + LOWER_TIME_MS) {
            return HopperPosition.TRANSITIONING;
        } else {
            if (wasAtTurret) {
                robot.shooter.queueIndex(robot.shooter.turretModule.currentRingsInTurret); // auto queue when we've lowered hopper down again (after being up)
            }
            wasAtTurret = false;

            return HopperPosition.LOWERED;
        }
    }

    public long msUntilHopperRaised() {
        long currentTime = robot.getCurrentTimeMilli();

        if (currentTime < deliveryStartTime + RAISE_TRANSITIONING_TIME_MS) {
            return (deliveryStartTime + RAISE_TRANSITIONING_TIME_MS) - currentTime;
        } else if (currentTime < deliveryStartTime + RAISE_TIME_MS + LOWER_CLEAR_SHOOTER_TIME_MS) {
            return (deliveryStartTime + RAISE_TIME_MS + LOWER_CLEAR_SHOOTER_TIME_MS) - currentTime;
        } else {
            return Long.MAX_VALUE;
        }
    }

    public HopperPosition getCurrentHopperPosition() {
        return calculateHopperPosition(deliveryStartTime, robot.getCurrentTimeMilli());
    }

    public int getRingsInHopper() {
        return ringsInHopper;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Hopper position: " + getCurrentHopperPosition());
        data.add("Rings in hopper: " + this.ringsInHopper);
        //data.add("DISTANCE: " + getRingCounterSensorReading());
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
