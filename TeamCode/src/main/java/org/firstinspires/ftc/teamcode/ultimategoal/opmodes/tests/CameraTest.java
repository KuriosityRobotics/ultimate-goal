package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCamera2View;
import org.opencv.core.Mat;

@TeleOp
public class CameraTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        JavaCamera2View view = new JavaCamera2View(hardwareMap.appContext, 0);
        view.setCvCameraViewListener(new CameraBridgeViewBase.CvCameraViewListener2() {
            @Override
            public void onCameraViewStarted(int width, int height) {

            }

            @Override
            public void onCameraViewStopped() {

            }

            @Override
            public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
                System.out.printf("Width of frame: %d, Height of frame: %d\n", inputFrame.rgba().width(), inputFrame.rgba().height());
                return inputFrame.rgba();
            }
        });
    }
}
