package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
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

//    public DistanceSensor distance;
    public AnalogInput distance;

    // Constants
    private static final double INDEX_OPEN_POSITION = 0.405;
    private static final double INDEX_PUSH_POSITION = 0.145;

    private static final int INDEXER_PUSHED_TIME_MS = 175;
    private static final int INDEXER_RETURNED_TIME_MS = 350;

    private static final double HOPPER_RAISED_POSITION = 0.965;
    private static final double HOPPER_LOWERED_POSITION = 0.63; // TODO find pos

    private static final int HOPPER_RAISE_TIME_MS = 500;
    private static final int HOPPER_LOWER_TIME_MS = 250;

    // Hopper position enum
    public enum HopperPosition {RAISED, LOWERED}

    public HopperModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {
        distance = robot.hardwareMap.get(AnalogInput.class, "distance");

        indexerServo = robot.getServo("indexerServo");
        hopperLinkage = robot.getServo("hopperLinkage");

        hopperLinkage.setPosition(HOPPER_LOWERED_POSITION);
        indexerServo.setPosition(INDEX_OPEN_POSITION);
    }

    private long indexTime = 0;
    private long hopperTransitionTime = 0;

    private HopperPosition oldHopperPosition = HopperPosition.LOWERED;
    int counter = 0;
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
        }

        boolean isDoneIndexing = currentTime > indexTime + INDEXER_PUSHED_TIME_MS;
        if (isDoneIndexing) {
            indexerServo.setPosition(INDEX_OPEN_POSITION);
        }

        if(getDistance() <= 55){
            counter++;
        }else{
            counter = 0;
        }

        if(counter >= 15){
            hopperLinkage.setPosition(HOPPER_RAISED_POSITION);
            counter = 0;
            robot.intakeModule.intakePower = -1;
        }

        if(getDistance() <= 75){
            counter2++;
        }else{
            counter2 = 0;
        }

        if(counter2 >= 15){
            robot.shooter.isFlyWheelOn = true;
            counter2 = 0;
        }
    }
    public int counter2;
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

    public boolean isIndexerFinishedPushing() {
        long currentTime = robot.getCurrentTimeMilli();
        return currentTime > indexTime + INDEXER_PUSHED_TIME_MS;
    }

    public boolean isDoneIndexing() {
        return isIndexerFinishedPushing() && !indexRing;
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

    public double getDistance(){
        double voltage_temp_average=0;

        for(int i=0; i < 2; i++)
        {
            voltage_temp_average += distance.getVoltage();

        }
        voltage_temp_average /= 2;

        //33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
        return (33.9 + -69.5*(voltage_temp_average) + 62.3*Math.pow(voltage_temp_average,2) + -25.4*Math.pow(voltage_temp_average,3) + 3.83*Math.pow(voltage_temp_average,4))*10;
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
        data.add("is indexer finished pushing: " + isIndexerFinishedPushing());
        data.add("is indexer returned: " + isIndexerReturned());
        data.add("is hopper at position: " + isHopperAtPosition());
        data.add("DISTANCE: " + getDistance());
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
