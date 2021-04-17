package org.firstinspires.ftc.teamcode.ultimategoal.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetryDump {
    Telemetry telemetry;
    private final ArrayList<TelemetryProvider> providers;
    public FtcDashboard dashboard;

    public TelemetryPacket packet;

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.providers = new ArrayList<>();

        this.dashboard = FtcDashboard.getInstance();
        this.packet = new TelemetryPacket();
    }

    public void registerProvider(TelemetryProvider provider) {
        providers.add(provider);
    }

    public void removeProvider(TelemetryProvider provider) {
        providers.remove(provider);
    }

    public void update() {
        StringBuilder out = new StringBuilder();
        for (TelemetryProvider provider : providers) {
            if (!provider.isOn()) {
                continue;
            }

            out.append("---").append(provider.getName()).append("---\n");

            for (String entry : provider.getTelemetryData()) {
                out.append(entry).append("\n");
            }

            out.append("\n");
        }
        telemetry.addLine(out.toString());
        telemetry.update();

        parseDashboardData();
        dashboard.sendTelemetryPacket(packet);
    }

    public void parseDashboardData(){
        for(TelemetryProvider provider : providers){
            if(provider.getDashboardData() != null) {
                for (String key : provider.getDashboardData().keySet()) {
                    sendDashboardData(key, provider.getDashboardData().get(key));
                }
            }
        }
    }

    public void sendDashboardData(String key, Object value){
        packet.put(key,value);
    }
}
