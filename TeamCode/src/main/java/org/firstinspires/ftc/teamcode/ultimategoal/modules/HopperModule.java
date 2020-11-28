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
    private static final double INDEX_OPEN_POSITION = 0.2075;
    private static final double INDEX_PUSH_POSITION = 0.3675;

    private static final int INDEXER_PUSHED_TIME_MS = 550;
    private static final int INDEXER_RETURNED_TIME_MS = 1100;

    private static final double HOPPER_RAISED_POSITION = 0.965;
    private static final double HOPPER_LOWERED_POSITION = 0.63; // TODO find pos

    private static final int HOPPER_RAISE_TIME_MS = 1000;
    private static final int HOPPER_LOWER_TIME_MS = 750;

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
        } else if (currentTime > initStartTime + Math.max((INDEXER_RETURNED_TIME_MS - INDEXER_PUSHED_TIME_MS), HOPPER_RAISE_TIME_MS)) {
            hopperLinkage.setPosition(HOPPER_LOWERED_POSITION);
        }

        return currentTime > initStartTime + Math.max((INDEXER_RETURNED_TIME_MS - INDEXER_PUSHED_TIME_MS), HOPPER_RAISE_TIME_MS) + HOPPER_LOWER_TIME_MS;
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
        if (hopperPosition == HopperPosition.RAISED && isHopperAtPosition()) {
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
    public boolean isHopperAtPosition() {
        long currentTime = robot.getCurrentTimeMilli();
        if (hopperPosition == HopperPosition.RAISED) {
            return currentTime > (hopperTransitionTime + HOPPER_RAISE_TIME_MS);
        } else {
            return currentTime > (hopperTransitionTime + HOPPER_LOWER_TIME_MS);
        }
    }

    public boolean isIndexerReturned() {
        long currentTime = robot.getCurrentTimeMilli();
        return currentTime > indexTime + INDEXER_RETURNED_TIME_MS;
    }

    public boolean isIndexerPushed() {
        long currentTime = robot.getCurrentTimeMilli();
        return currentTime > indexTime + INDEXER_PUSHED_TIME_MS;
    }

    public boolean isDoneIndexing() {
        return isIndexerReturned() && !indexRing;
    }

    /**
     * Attempt to index a ring. If the indexer is currently indexing, nothing will happen. Whether
     * or not the command was successfully executed is returned.
     *
     * @return Whether or not the index command will be processed.
     */
    public boolean requestRingIndex() {
        if (!indexRing) {
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
        data.add("Hopper position: " + hopperPosition.toString());
        data.add("Will index: " + indexRing);
        data.add("--");
        data.add("index time: " + indexTime);
        data.add("current time: " + robot.getCurrentTimeMilli());
        data.add("is indexer returned: " + isIndexerReturned());
        data.add("is indexer pushed: " + isIndexerPushed());
        data.add("is hopper at positoin: " + isHopperAtPosition());
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
