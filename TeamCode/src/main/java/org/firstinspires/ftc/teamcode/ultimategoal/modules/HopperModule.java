package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class HopperModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public HopperPosition hopperPosition = HopperPosition.LOWERED;
    public boolean indexRing = false;

    // Servos
    private Servo indexerServo;
    private Servo hopperLinkage;

    // Constants
    private static final double INDEX_OPEN_POSITION = .85;
    private static final double INDEX_PUSH_POSITION = .68;

    private static final int INDEXER_PUSHED_TIME_MS = 600;
    private static final int INDEXER_RETURNED_TIME_MS = 1200;

    private static final double HOPPER_RAISED_POSITION = 0.96;
    private static final double HOPPER_LOWERED_POSITION = 0.96; // TODO find pos

    private static final int HOPPER_RAISE_TIME_MS = 2000;
    private static final int HOPPER_LOWER_TIME_MS = 2000;

    // Hopper position enum
    public enum HopperPosition {RAISED, LOWERED}

    public HopperModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        indexerServo = robot.getServo("indexerServo");
        hopperLinkage = robot.getServo("hopperLinkage");
    }

    long initStartTime = 0;

    public boolean initCycle() {
        long currentTime = SystemClock.elapsedRealtime();

        if (initStartTime == 0) {
            hopperLinkage.setPosition(HOPPER_RAISED_POSITION);
            indexerServo.setPosition(INDEX_OPEN_POSITION);

            initStartTime = currentTime;
        } else if (currentTime > initStartTime + (INDEXER_RETURNED_TIME_MS - INDEXER_PUSHED_TIME_MS)) {
            hopperLinkage.setPosition(HOPPER_LOWERED_POSITION);

            initStartTime = currentTime;
        }

        if (currentTime > initStartTime + (INDEXER_RETURNED_TIME_MS - INDEXER_PUSHED_TIME_MS) + HOPPER_LOWER_TIME_MS) {
            return true;
        } else {
            return false;
        }
    }

    private long indexTime = 0;
    private long hopperTransitionTime = 0;

    private HopperPosition oldHopperPosition = HopperPosition.LOWERED;

    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        if (hopperPosition != oldHopperPosition) {
            if (hopperPosition == HopperPosition.RAISED) {
                hopperLinkage.setPosition(HOPPER_RAISED_POSITION);
            } else {
                hopperLinkage.setPosition(HOPPER_LOWERED_POSITION);
            }

            oldHopperPosition = hopperPosition;
            hopperTransitionTime = currentTime;
        }

        // Index logic if the hopper is up
        if (hopperPosition == HopperPosition.RAISED && isAtPosition()) {
            boolean indexerReturned = currentTime > indexTime + INDEXER_RETURNED_TIME_MS;
            if (indexRing && indexerReturned) {
                indexerServo.setPosition(INDEX_PUSH_POSITION);
                indexTime = currentTime;
                indexRing = false;
            }

            boolean isDoneIndexing = currentTime > indexTime + INDEXER_PUSHED_TIME_MS;
            if (isDoneIndexing) {
                indexerServo.setPosition(INDEX_OPEN_POSITION);
            }
        }
    }

    public void switchHopperPosition() {
        if (hopperPosition == HopperPosition.LOWERED) {
            hopperPosition = HopperPosition.RAISED;
        } else {
            hopperPosition = HopperPosition.LOWERED;
        }
    }

    /**
     * Returns whether or not the hopper is at the position specified.
     *
     * @return whether or not the hopper is at the position specified.
     */
    public boolean isAtPosition() {
        long currentTime = robot.getCurrentTimeMilli();
        if (hopperPosition == HopperPosition.RAISED) {
            return currentTime > (hopperTransitionTime + HOPPER_RAISE_TIME_MS);
        } else {
            return currentTime > (hopperTransitionTime + HOPPER_LOWER_TIME_MS);
        }
    }

    /**
     * Attempt to index a ring. If the indexer is currently indexing, nothing will happen. Whether
     * or not the command was successfully executed is returned.
     *
     * @return Whether or not the index command will be processed.
     */
    public boolean requestRingIndex() {
        if (robot.getCurrentTimeMilli() > indexTime + INDEXER_RETURNED_TIME_MS && robot.shooter.isUpToSpeed()) {
            indexRing = true;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Hopper position" + hopperPosition.toString());
        data.add("Will index: " + indexRing);
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
