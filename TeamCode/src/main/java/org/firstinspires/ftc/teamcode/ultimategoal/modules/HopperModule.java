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
    public HopperPosition currentHopperPosition;

    // Servos
    private Servo hopperLinkage;

    // Helpers
    private long deliveryStartTime = 0;

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
        currentHopperPosition = HopperPosition.LOWERED;
    }

    @Override
    public void initModules() {
        hopperLinkage = robot.getServo("hopper");

        hopperLinkage.setPosition(LINKAGE_LOWERED_POSITION);
    }


    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();


        currentHopperPosition = calculateHopperPosition(this.deliveryStartTime, currentTime); // do the thing

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
        } else {
            hopperLinkage.setPosition(LINKAGE_LOWERED_POSITION);
        }
    }

    private HopperPosition calculateHopperPosition(long deliveryStartTime, long currentTime) {
        if (currentTime < deliveryStartTime + RAISE_TRANSITIONING_TIME_MS) {
            return HopperPosition.TRANSITIONING;
        } else if (currentTime < deliveryStartTime + RAISE_TIME_MS + LOWER_CLEAR_SHOOTER_TIME_MS) {
            return HopperPosition.AT_TURRET;
        } else if (currentTime < deliveryStartTime + RAISE_TIME_MS + LOWER_TIME_MS) {
            return HopperPosition.TRANSITIONING;
        } else {
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

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Hopper position: " + getCurrentHopperPosition());
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
