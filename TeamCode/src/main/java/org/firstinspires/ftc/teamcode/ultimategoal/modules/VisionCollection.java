package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import android.hardware.Camera;
import android.hardware.camera2.CameraCaptureSession;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.vision.BlueHighGoalDetector;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.vision.VisionModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;

public class VisionCollection implements Module, TelemetryProvider {
    VideoCapture videoCapture;

    Robot robot;
    boolean isOn;

    VisionModule[] visionModules;

    public BlueHighGoalDetector highGoalDetector;

    public VisionCollection(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModule() {
        highGoalDetector = new BlueHighGoalDetector(true);

        visionModules = new VisionModule[]{
                highGoalDetector
        };

        videoCapture = new VideoCapture();
        videoCapture.open(0);
        System.out.println(videoCapture.isOpened());

        Camera

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
        ArrayList<String> data = new ArrayList<>();
        data.add(highGoalDetector.getCameraPosition().toString());
        data.add(highGoalDetector.getRobotPosition().toString());
        return data;
    }

    @Override
    public String getName() {
        return "VisionModule";
    }
}