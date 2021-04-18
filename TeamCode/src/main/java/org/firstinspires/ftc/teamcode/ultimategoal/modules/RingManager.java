package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class RingManager implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // Sensors
    AnalogInput intakeDistance;

    // States
    public boolean willAutoRaise;
    public boolean autoShootRings;
    public int autoRaiseThreshold = 1;

    // Data
    public int ringsInIntake; // will implement later
    public int ringsInHopper;
    public int ringsInShooter;

    // for incoming ring detection
    private long lastLoopTime;
    private boolean seeingRing = false;
    private int distanceSensorPasses = 0;

    private static final int INTAKE_STOP_DELAY_TIME = 600;

    public RingManager(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);

        ringsInIntake = 0;
        ringsInHopper = 0; // will change to 3 later b/c autonomous starts w 3 rings
        ringsInShooter = 0;

        willAutoRaise = true;
        autoShootRings = true;

        this.lastLoopTime = robot.getCurrentTimeMilli();
    }

    public void initModules() {
        intakeDistance = robot.hardwareMap.get(AnalogInput.class, "distance");
    }

    public void update() {
        if (ringsInShooter < 0) {
            ringsInShooter = 0;
        }

        countPassingRings();
        detectHopperDelivery();
        detectShotRings();
    }

    private void countPassingRings() {
        long currentTime = robot.getCurrentTimeMilli();
        double voltage = intakeDistance.getVoltage();

        if (Math.abs(2.2 - voltage) < Math.abs(1.1 - voltage)) {
            seeingRing = true;
        } else if (seeingRing) { // we saw a ring but now we don't
            int increment = 1;

            // If we had a really large loop time and an odd number of passes, just round up
            if (currentTime - lastLoopTime > 200 && distanceSensorPasses % 2 == 1) {
                increment++;
            }

            if (robot.intakeModule.intakeBottom.getPower() > 0) {// outtaking or intaking ?
                distanceSensorPasses += increment;
            } else {
                distanceSensorPasses -= increment;
            }

            seeingRing = false;
        }

        ringsInHopper = (int) (distanceSensorPasses / 2.0);

        if (ringsInHopper >= autoRaiseThreshold && willAutoRaise) {
            robot.shooter.deliverRings();
        }

        lastLoopTime = currentTime;
    }

    private HopperModule.HopperPosition oldHopperPosition = HopperModule.HopperPosition.LOWERED;

    public void detectHopperDelivery() {
        HopperModule.HopperPosition currentHopperPosition = robot.shooter.getHopperPosition();

        // when hopper push is finished move rings
        if (oldHopperPosition == HopperModule.HopperPosition.TRANSITIONING && currentHopperPosition == HopperModule.HopperPosition.LOWERED) {
            ringsInShooter = ringsInHopper;
            distanceSensorPasses = 0;

            if (autoShootRings) {
                robot.shooter.queueIndex(robot.ringManager.ringsInShooter);
            }
        }

        oldHopperPosition = currentHopperPosition;
    }

    private ShooterModule.IndexerPosition oldIndexerPosition = ShooterModule.IndexerPosition.RETRACTED;

    public void detectShotRings() {
        ShooterModule.IndexerPosition currentIndexerPosition = robot.shooter.getIndexerPosition();

        // when indexer has returned from an index the shooter loses a ring
        if (oldIndexerPosition == ShooterModule.IndexerPosition.PUSHED && currentIndexerPosition == ShooterModule.IndexerPosition.RETRACTED) {
            ringsInShooter--;
        }

        oldIndexerPosition = currentIndexerPosition;
    }

    public void resetRingCounters() {
        distanceSensorPasses = 0;
        ringsInIntake = 0;
        ringsInHopper = 0;
        ringsInShooter = 0;
    }

    public boolean canIntake() {

        long currentTime = robot.getCurrentTimeMilli();
        if (ringsInHopper + ringsInShooter < autoRaiseThreshold) {
            return true;
        } else {
            return robot.shooter.getHopperPosition() == HopperModule.HopperPosition.LOWERED;
            // TODO move this out, also we can intake if we have less than three and the hopper is transitioning until the ring hits the sensor
        }
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Ring passes: " + distanceSensorPasses);
        data.add("Rings in intake: " + ringsInIntake);
        data.add("Rings in hopper: " + ringsInHopper);
        data.add("Rings in shooter: " + ringsInShooter);
        data.add("Will Auto Raise:  " + willAutoRaise);
        data.add("Will Auto Shoot: " + autoShootRings);
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
}
