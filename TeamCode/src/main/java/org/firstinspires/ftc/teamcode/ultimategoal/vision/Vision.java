package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests.VisionTest;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision {
    OpenCvWebcam webcam;

    public void init(Robot robot) {
        VisionPipeline pipeline = new VisionPipeline();
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        });
    }

    class VisionPipeline extends OpenCvPipeline {
        public RingStackLocator.TargetZone targetZone = RingStackLocator.TargetZone.TARGET_ZONE_UNKNOWN;
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            this.targetZone = RingStackLocator.processFrame(input, true);
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}
