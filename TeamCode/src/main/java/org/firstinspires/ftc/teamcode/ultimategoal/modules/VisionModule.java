package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.GoalFinder;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionModule implements Module {
    OpenCvWebcam webcam;
    Robot robot;
    GoalFinder goalFinder;
    boolean isOn;

    public VisionModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;
    }

    @Override
    public void initModules() {
        goalFinder = new GoalFinder();
        try {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
            webcam.setPipeline(goalFinder);

            webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            webcam.openCameraDeviceAsync(() -> {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            });
        } catch (Throwable nonpog) { // If this fails, it's okay; the default value is null, and the robot will try to work out how to aim w/o the cam
            nonpog.printStackTrace();
        }
    }

    @Override
    public boolean initCycle() {
        return true;
    }

    @Override
    public void update() {
        goalFinder.isBlue = robot.shooter.target.name().contains("BLUE");
    }


    @Override
    public boolean isOn() {
        return this.isOn;
    }

    @Override
    public String getName() {
        return "VisionModule";
    }

    public GoalFinder.GoalLocationData getLocationData() {
        return this.goalFinder.getLocationData();
    }
}
