package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.wrappers.AnalogDistance;

import java.util.ArrayList;
import java.util.HashMap;

public class RingManager implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // Sensors
    AnalogInput intakeCounterDistance;
    //    AnalogInput intakeEntranceDistance;
    AnalogDistance intakeEntranceDistance;

    // States
    public boolean autoRaise;
    public boolean autoShootRings;
    public int autoRaiseThreshold = 1;
    public boolean intakeFollowThrough;

    // Constants
    private static final int HOPPER_DELIVERY_DELAY = 700;

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
    private int forwardDistanceSensorPasses = 0;

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
        intakeFollowThrough = true;
    }

    public void initModules() {
        intakeCounterDistance = robot.hardwareMap.get(AnalogInput.class, "distance");
        intakeEntranceDistance = new AnalogDistance(robot.hardwareMap.get(AnalogInput.class, "distanceCenter"));
    }

    public void update() {
        detectShotRings();
        detectHopperDelivery();
        countPassingRings();

        stopIntakeLogic();
    }

    private void countPassingRings() {
        long currentTime = robot.getCurrentTimeMilli();
        double voltage = intakeCounterDistance.getVoltage();
        lastSensorReading = voltage;

        Log.v("ringmanager", "intakedistance: " + voltage);

        if (voltage > 1.55) {
            seeingRing = true;
        } else if (seeingRing) { // we saw a ring but now we don't
            if (robot.intakeModule.intakeTop.getPower() >= 0) {// outtaking or intaking ?
                distanceSensorPasses += 1;
                forwardDistanceSensorPasses += 1;
            } else {
                distanceSensorPasses -= 1;
            }

            seeingRing = false;
        }

        ringsInHopper = distanceSensorPasses;

        boolean readyToAutoDeliver = autoRaise && ringsInHopper >= autoRaiseThreshold;
        boolean hopperReady = robot.shooter.getCurrentHopperPosition() == HopperModule.HopperPosition.LOWERED;
        boolean shooterNotFull = ringsInShooter + ringsInHopper <= 3;
        if (readyToAutoDeliver && hopperReady && shooterNotFull) {
            if (!deliverRings) {
                deliverRings = true;
                deliverDelayStartTime = currentTime;
            } else if (seeingRing) { // if we're going to deliver BUT WAIT! there's another ring!
                deliverDelayStartTime = currentTime;
            }
        } else {
            deliverRings = false;
        }

        if (deliverRings && currentTime >= deliverDelayStartTime + HOPPER_DELIVERY_DELAY) {
            deliverRings = false;
            robot.shooter.deliverRings();
            Log.v("ringmanager", "delivering rings");
        }
    }

    private HopperModule.HopperPosition oldHopperPosition = HopperModule.HopperPosition.LOWERED;

    public void detectHopperDelivery() {
        HopperModule.HopperPosition currentHopperPosition = robot.shooter.getCurrentHopperPosition();

        // when hopper push is finished move rings
        if (oldHopperPosition == HopperModule.HopperPosition.TRANSITIONING && currentHopperPosition == HopperModule.HopperPosition.LOWERED) {
            ringsInShooter = ringsInShooter + ringsInHopper;
            distanceSensorPasses = 0;
            ringsInHopper = 0;
            forwardDistanceSensorPasses = 0;
//            seenRingSinceRaise = false;

            if (autoShootRings) {
//                robot.shooter.clearIndexes();
                robot.shooter.queueIndex(ringsInShooter);
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
                ringsInShooter = ringsInShooter - 1;
                totalRingsShot = totalRingsShot + 1;
            }
        }

        oldIndexerPosition = currentIndexerPosition;
    }

    boolean entranceSeeingRing = false;
    boolean seenRingSinceRaise = false;

    private void stopIntakeLogic() {
        double distance = intakeEntranceDistance.getSensorReading();

        Log.v("ringmanager", "entrance sensor: " + distance);

        if (distance < 80) {
            entranceSeeingRing = true;
        } else {
            entranceSeeingRing = false;
        }

//        if (robot.shooter.getCurrentHopperPosition() != HopperModule.HopperPosition.LOWERED || deliverRings || robot.shooter.deliveryQueued()) {
//            seenRingSinceRaise = seenRingSinceRaise || (entranceSeeingRing && !seeingRing);
//            robot.intakeModule.stopIntake = seenRingSinceRaise;
//        } else {
//            robot.intakeModule.stopIntake = false;
//        }
//
//        if (robot.shooter.getCurrentHopperPosition() == HopperModule.HopperPosition.LOWERED) {
//            seenRingSinceRaise = false;
//        }

        if (entranceSeeingRing) {
            robot.intakeModule.stopIntake = true;
        }
    }

    public void resetRingCounters() {
        distanceSensorPasses = 0;
        forwardDistanceSensorPasses = 0;
        ringsInHopper = 0;
        ringsInShooter = 0;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("counter voltage: " + lastSensorReading);
        data.add("Ring passes: " + distanceSensorPasses);
        data.add("deliverRings: " + deliverRings);
        data.add("Rings in hopper: " + ringsInHopper + ", Rings in shooter: " + ringsInShooter);
        data.add("Will Auto Raise:  " + autoRaise + ", Will Auto Shoot: " + autoShootRings);
        return data;
    }

    @Override
    public HashMap<String, Object> getDashboardData() {
        HashMap<String, Object> data = new HashMap<>();
        data.put("rings in hopper", ringsInHopper);
        return data;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "RingManager";
    }

    public int getDistanceSensorPasses() {
        return distanceSensorPasses;
    }

    public void addDistanceSensorPasses(int num) {
        distanceSensorPasses += num;
    }

    public int getForwardDistanceSensorPasses() {
        return forwardDistanceSensorPasses;
    }

    public int getRingsInHopper() {
        return ringsInHopper;
    }

    public int getRingsInShooter() {
        return ringsInShooter;
    }

    public void addRingsInShooter(int num) {
        ringsInShooter += num;
    }

    public int getTotalRingsShot() {
        return totalRingsShot;
    }

    public int getRingsInSystem() {
        return ringsInHopper + ringsInShooter;
    }

    public boolean getSeeingRing() {
        return seeingRing;
    }

    public boolean getDeliverRings() {
        return deliverRings;
    }
}
