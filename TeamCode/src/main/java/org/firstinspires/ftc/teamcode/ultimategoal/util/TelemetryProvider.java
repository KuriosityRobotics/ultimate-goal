package org.firstinspires.ftc.teamcode.ultimategoal.util;

import java.util.ArrayList;

public interface TelemetryProvider {
    ArrayList<String> getTelemetryData();

    default boolean isOn() {
        return true;
    }

    String getName();
}
