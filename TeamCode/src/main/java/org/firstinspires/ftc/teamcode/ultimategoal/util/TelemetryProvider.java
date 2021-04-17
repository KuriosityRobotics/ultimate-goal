package org.firstinspires.ftc.teamcode.ultimategoal.util;

import java.util.ArrayList;
import java.util.HashMap;

public interface TelemetryProvider {
    ArrayList<String> getTelemetryData();
    default HashMap<String, Object> getDashboardData(){
        return null;
    }

    default boolean isOn() {
        return true;
    }

    String getName();
}
