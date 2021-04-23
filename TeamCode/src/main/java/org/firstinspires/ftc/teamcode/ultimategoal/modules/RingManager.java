package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.HashMap;

public class RingManager implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // Sensors
    AnalogInput intakeDistance;

    // States
    public boolean autoRaise;
    public boolean autoShootRings;
    public int autoRaiseThreshold = 1;

    // Data
    private int ringsInHopper;
    private int ringsInShooter;
    private int totalRingsShot;

    public double lastSensorReading;

    // for incoming ring detection
    private boolean deliverRings;
    private long deliverDelayStartTime;
    private boolean seeingRing = false;
    private int distanceSensorPasses = 0;
    private boolean seenRingSinceStartDelivery = false;

    private static final int HOPPER_DELIVERY_DELAY = 750;

    public RingManager(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        ringsInHopper = 0; // will change to 3 later b/c autonomous starts w 3 rings
        ringsInShooter = 0;
        totalRingsShot = 0;

        autoRaise = true;
        autoShootRings = true;

        deliverRings = false;
    }

    public void initModules() {
        intakeDistance = robot.hardwareMap.get(AnalogInput.class, "distance");
    }

    public void update() {
        detectHopperDelivery();
        countPassingRings();
        detectShotRings();

        stopIntakeLogic();

        Log.v("ringmanager", "inshooter: " + ringsInShooter);
        Log.v("ringmanager", "inhopper: " + ringsInHopper);
    }

    private void countPassingRings() {
        long currentTime = robot.getCurrentTimeMilli();
        double voltage = intakeDistance.getVoltage();
        lastSensorReading = voltage;

        if (voltage > 1.25) {
            seeingRing = true;
        } else if (seeingRing) { // we saw a ring but now we don't
            if (robot.intakeModule.intakeBottom.getPower() > 0) {// outtaking or intaking ?
                distanceSensorPasses += 1;
            } else {
                distanceSensorPasses -= 1;
            }

            seeingRing = false;
        }

        ringsInHopper = (int) Math.ceil(distanceSensorPasses / 2.0);

        if (getRingsInHopper() >= autoRaiseThreshold && autoRaise) {
            if (!deliverRings && robot.shooter.getHopperPosition() == HopperModule.HopperPosition.LOWERED) {
                deliverRings = true;
                deliverDelayStartTime = currentTime;
            } else if (deliverRings && seeingRing) { // if we're going to deliver BUT WAIT! there's another ring!
                deliverDelayStartTime = currentTime;
            }
        }

        if (seeingRing && robot.shooter.getHopperPosition() != HopperModule.HopperPosition.LOWERED) {
            seenRingSinceStartDelivery = true;
        }

        if (deliverRings && currentTime >= deliverDelayStartTime + HOPPER_DELIVERY_DELAY) {
            deliverRings = false;
            seenRingSinceStartDelivery = false;
            robot.shooter.deliverRings();
        }
    }

    private HopperModule.HopperPosition oldHopperPosition = HopperModule.HopperPosition.LOWERED;

    public void detectHopperDelivery() {
        HopperModule.HopperPosition currentHopperPosition = robot.shooter.getHopperPosition();

        // when hopper push is finished move rings
        if (oldHopperPosition == HopperModule.HopperPosition.TRANSITIONING && currentHopperPosition == HopperModule.HopperPosition.LOWERED) {
            ringsInShooter = ringsInShooter + ringsInHopper;
            distanceSensorPasses = 0;

            if (autoShootRings) {
                robot.shooter.queueIndex(ringsInShooter);
                Log.v("ringmanager", "auto shot: " + ringsInShooter);
            }
        }

        oldHopperPosition = currentHopperPosition;
    }

    private ShooterModule.IndexerPosition oldIndexerPosition = ShooterModule.IndexerPosition.RETRACTED;

    public void detectShotRings() {
        ShooterModule.IndexerPosition currentIndexerPosition = robot.shooter.getIndexerPosition();

        // when indexer has returned from an index the shooter loses a ring
        if (oldIndexerPosition == ShooterModule.IndexerPosition.PUSHED && currentIndexerPosition == ShooterModule.IndexerPosition.RETRACTED) {
            if (ringsInShooter > 0) {
                ringsInShooter = getRingsInShooter() - 1;
                totalRingsShot = getTotalRingsShot() + 1;
            }
        }

        oldIndexerPosition = currentIndexerPosition;
    }

    boolean fullRings;
    long fullRingsTime = 0;

    private void stopIntakeLogic() {
//        if (ringsInHopper + ringsInShooter >= 3) {
//            long currentTime = robot.getCurrentTimeMilli();
//
//            if (!fullRings) {
//                fullRingsTime = currentTime;
//            }
//
//            fullRings = true;
//
//            if (currentTime > fullRingsTime + 1000) {
//                robot.intakeModule.stopIntake = true;
//                return;
//            }
//        } else {
//            fullRings = false;
//        }

        if (robot.shooter.targetHopperPosition() != HopperModule.HopperPosition.LOWERED) {
            robot.intakeModule.stopIntake = seenRingSinceStartDelivery;
        } else if (deliverRings) { // waiting to raise the hopper
            robot.intakeModule.intakePower = 1;
        } else {
            robot.intakeModule.stopIntake = false;
        }
    }

    public void resetRingCounters() {
        distanceSensorPasses = 0;
        ringsInHopper = 0;
        ringsInShooter = 0;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Ring passes: " + distanceSensorPasses);
        data.add("deliverRings: " + deliverRings);
        data.add("Rings in hopper: " + getRingsInHopper() + ", Rings in shooter: " + getRingsInShooter());
        data.add("Will Auto Raise:  " + autoRaise + ", Will Auto Shoot: " + autoShootRings);
        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("rings in hopper", getRingsInHopper());
        return data;
    }

    public void setDistanceSensorPasses(int distanceSensorPasses) {
        this.distanceSensorPasses = distanceSensorPasses;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "RingManager";
    }

    public int getRingsInHopper() {
        return ringsInHopper;
    }

    public int getRingsInShooter() {
        return ringsInShooter;
    }

    public int getTotalRingsShot() {
        return totalRingsShot;
    }

    public boolean getSeeingRing() {
        return seeingRing;
    }
}
