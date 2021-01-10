package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.vision.HighGoalDetector;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.vision.VisionModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;

public class VisionCollection implements Module, TelemetryProvider {
    VideoCapture videoCapture;

    Robot robot;
    boolean isOn;

    VisionModule[] visionModules;

    public HighGoalDetector highGoalDetector;

    public VisionCollection(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModule() {
        highGoalDetector = new HighGoalDetector(true);

        visionModules = new VisionModule[]{
                highGoalDetector
        };

        videoCapture = new VideoCapture();
        videoCapture.open(0); // TODO find right cam index
    }

    @Override
    public void update() {
        // Pull camera frame into a Mat
        Mat image = new Mat();
        videoCapture.read(image);

        for (VisionModule visionModule : visionModules) {
            if (visionModule.isOn()) {
                visionModule.update(image);
            }
        }
    }

    @Override
    public boolean isOn() {
        return this.isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        return null;
    }

    @Override
    public String getName() {
        return "VisionModule";
    }
}