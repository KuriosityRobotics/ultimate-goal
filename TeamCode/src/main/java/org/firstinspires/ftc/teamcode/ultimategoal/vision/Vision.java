package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.ConditionVariable;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ExecutorService;

public class Vision {
    private final String VUFORIA_KEY = "AblAnwD/////AAABmRHXA7f65ErFhMbZmr+8xjArZA83Y+l2nal3r90Dmzl6nc0hUj+zgUCK3sF8PxkhDDJkMsSJSl05Q3U0Bjz6HeydKoGMwsvF8x2IbUto/6gbCm8WqDkvfBjzDeVL5Y3XCkczOi1F8dmNt1JkJQdX4bJokLrzEBQnQOF6mwxI22M2eSobTgyHSrZk4hl6jTXVSO9ckVtMfVjV/pryDQMnnJDFMQ/64u+uhxtnsMZKgd9UlORAMwsSL9Wwk1ixoWeUsLzZS4w/5b4GbupBTsY/teWORJo0AulqTI+rCJRhKzQcZRlG7v5jt2f3es7y0uXbxT5QrQ06tNmvhEfjAIduF5eSbPZ/3QjQnFgtRuyMW0ix";

    public enum TargetGoal {
        A, B, C, UNKNOWN
    }

    WebcamName webcamName;

    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    int captureCounter = 0;

    private VuforiaLocalizer vuforia;

    private final LinearOpMode linearOpMode;

    public Vision(LinearOpMode linearOpmode) {
        this.linearOpMode = linearOpmode;
        initVision();
    }

    public void initVision() {
        initVuforia();
    }

    public static double calcOrangeValue(Bitmap bitmap, int x, int y, int width, int height) {
        double sum = 0;
        int endX = x + width;
        int endY = y + height;
        int intColor;
        int redGreen;

        for (int i = x; i < endX; i++) {
            for (int j = y; j < endY; j++) {
                intColor = bitmap.getPixel(i, j);
                redGreen = Color.red(intColor) * 20;
                redGreen -= Color.blue(intColor) * 20;
                sum += redGreen;
            }
        }

        return sum / (width * height);
    }

    public void captureFrameToFile(Bitmap bitmap) {
        if (bitmap != null) {
            File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
            try {
                FileOutputStream outputStream = new FileOutputStream(file);
                try {
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                } finally {
                    outputStream.close();
                    Log.d("Vision", "captured %s" + file.getAbsolutePath());
                }
            } catch (IOException e) {
                RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
            }
        }
    }

    public void updateBitmapWithBoundingBoxes(Bitmap bitmap, int x, int y, int width, int height) {
        int endX = x + width;
        int endY = y + height;

        for (int i = x; i < endX; i++) {
            for (int j = y; j < endY; j++) {
                if (i == x || i == endX - 1) {
                    bitmap.setPixel(i, j, Color.argb(1, 0, 0, 0));
                } else if (j == y || j == endY - 1) {
                    bitmap.setPixel(i, j, Color.argb(1, 0, 0, 0));
                }
            }
        }
    }

    public TargetGoal runDetection() {
        if (vuforia != null) {
            final TargetGoal[] resultTargetGoal = {TargetGoal.UNKNOWN};

            final long startTime = SystemClock.elapsedRealtime();

            final ExecutorService executorService = ThreadPool.getDefault();
// && SystemClock.elapsedRealtime() - startTime <= 100
            while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime() - startTime <= 100) {
                final ConditionVariable resultAvaliable = new ConditionVariable(false);

                vuforia.getFrameOnce(Continuation.create(executorService, new Consumer<Frame>() {
                    @Override
                    public void accept(Frame frame) {
                        Bitmap bitmap = vuforia.convertFrameToBitmap(frame);

                        if (bitmap != null) {
                            double percentageOrange = calcOrangeValue(bitmap, 170, 335, 85, 60);

                            Log.d("Vision", Double.toString(percentageOrange));
                            if (percentageOrange >= 1500) {
                                resultTargetGoal[0] = TargetGoal.C;
                            } else if (percentageOrange < 1500 && percentageOrange > 410) {
                                resultTargetGoal[0] = TargetGoal.B;
                            } else {
                                resultTargetGoal[0] = TargetGoal.A;
                            }

                            //                            updateBitmapWithBoundingBoxes(bitmap,105, 287, 128, 100);
                            if (linearOpMode.gamepad1.a) {
                                updateBitmapWithBoundingBoxes(bitmap, 170, 335, 85, 60);

                                captureFrameToFile(bitmap);
                            }
                            //                            captureFrameToFile(bitmap);
                        }


                        resultAvaliable.open();
                    }
                }));

                resultAvaliable.block();
            }

            Log.d("Vision", "RESULT " + resultTargetGoal[0].toString());

            return resultTargetGoal[0];
        }
        return TargetGoal.UNKNOWN;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        try {
            webcamName = linearOpMode.hardwareMap.get(WebcamName.class, "Webcam");

            int cameraMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.cameraName = webcamName;

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            Log.d("CAMERA", vuforia.getCameraName().toString());
            vuforia.enableConvertFrameToBitmap();
            linearOpMode.telemetry.addLine("DONE INITING VISION");
            linearOpMode.telemetry.update();
        } catch (Exception e) {
            Log.d("VUFORIA", e.toString());
        }
    }
}
