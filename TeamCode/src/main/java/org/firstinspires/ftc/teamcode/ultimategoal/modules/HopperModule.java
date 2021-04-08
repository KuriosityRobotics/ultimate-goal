package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class HopperModule implements Module, TelemetryProvider {
    Robot robot;
    boolean isOn;

    // States
    public boolean deliverRings;

    // Data
    private int ringsInHopper;

    // Servos
    private Servo hopperLinkage;

    // Sensors
//    private AnalogInput ringCounterSensor;

    // Helpers
    private long deliveryStartTime = 0;

    // Constants
    private static final double LINKAGE_LOWERED_POSITION = 0;
    private static final double LINKAGE_RAISED_POSITION = 0.36;

    private static final int RAISE_TIME_MS = 0; // from lowered to apex
    private static final int RAISE_TRANSITIONING_TIME_MS = 0; // from lowered to interfering with shooter
    private static final int LOWER_TIME_MS = 0; // from apex to lowered
    private static final int LOWER_CLEAR_SHOOTER_TIME_MS = 0; // from apex to no longer interfering with shooter

//    private static final int SECOND_RING_SENSOR_THRESHOLD = 75;
//    private static final int THIRD_RING_SENSOR_THRESHOLD = 55;
//
//    private static final int SENSOR_BUFFER_CYCLES = 15;

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
//        ringCounterSensor = robot.hardwareMap.get(AnalogInput.class, "distance");

        hopperLinkage = robot.getServo("hopperLinkage");

        hopperLinkage.setPosition(LINKAGE_LOWERED_POSITION);
    }

    @Override
    public void update() {
        long currentTime = robot.getCurrentTimeMilli();

        HopperPosition currentHopperPosition = calculateHopperPosition(this.deliveryStartTime, currentTime);

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

//        countRingsInHopper();
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

//    private int thirdRingCounter = 0;
//    private int secondRingCounter = 0;

//    private void countRingsInHopper() {
//        double sensorReading = getRingCounterSensorReading();
//
//        if (sensorReading <= THIRD_RING_SENSOR_THRESHOLD) {
//            if (thirdRingCounter != Integer.MAX_VALUE) {
//                thirdRingCounter++;
//            }
//        } else {
//            thirdRingCounter = 0;
//        }
//
//        if (sensorReading <= SECOND_RING_SENSOR_THRESHOLD) {
//            if (secondRingCounter != Integer.MAX_VALUE) {
//                secondRingCounter++;
//            }
//        } else {
//            secondRingCounter = 0;
//        }
//
//        if (thirdRingCounter >= SENSOR_BUFFER_CYCLES) {
//            ringsInHopper = 3;
//        } else if (secondRingCounter >= SENSOR_BUFFER_CYCLES) {
//            ringsInHopper = 2;
//        } else {
//            ringsInHopper = 0;
//        }
//    }
//
//    private double getRingCounterSensorReading() {
//        double voltage_temp_average = 0;
//
//        for (int i = 0; i < 2; i++) {
//            voltage_temp_average += ringCounterSensor.getVoltage();
//        }
//        voltage_temp_average /= 2;
//
//        //33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
//        return (33.9 + -69.5 * (voltage_temp_average) + 62.3 * Math.pow(voltage_temp_average, 2) + -25.4 * Math.pow(voltage_temp_average, 3) + 3.83 * Math.pow(voltage_temp_average, 4)) * 10;
//    }


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
        return data;
    }

    @Override
    public String getName() {
        return "HopperModule";
    }
}
