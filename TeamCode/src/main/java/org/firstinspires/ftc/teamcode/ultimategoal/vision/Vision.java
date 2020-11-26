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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ExecutorService;

public class Vision {
    private final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

    public enum Location {
        A, B, C, UNKNOWN
    }

    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    int captureCounter = 0;

    private VuforiaLocalizer vuforia;

    private LinearOpMode linearOpMode;

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
                redGreen = Color.red(intColor)*20;
                redGreen -= Color.blue(intColor)*20;
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

    public Location runDetection() {
        if (vuforia != null) {
            final Location[] resultLocation = {Location.UNKNOWN};

            final long startTime = SystemClock.elapsedRealtime();

            final ExecutorService executorService = ThreadPool.getDefault();

            while (linearOpMode.opModeIsActive() && SystemClock.elapsedRealtime()-startTime<=250) {
                final ConditionVariable resultAvaliable = new ConditionVariable(false);

                vuforia.getFrameOnce(Continuation.create(executorService, new Consumer<Frame>() {
                    @Override
                    public void accept(Frame frame) {
                        Bitmap bitmap = vuforia.convertFrameToBitmap(frame);

                        if (bitmap != null) {
                            double percentageOrange = calcOrangeValue(bitmap,185,465,185,145);
                            //360 600

                            Log.d("Vision",Double.toString(percentageOrange));
                            if(percentageOrange > 900){
                                resultLocation[0] = Location.C;
                            }else if(percentageOrange < 900 && percentageOrange > 0){
                                resultLocation[0] = Location.B;
                            }else{
                                resultLocation[0] = Location.A;
                            }
//                            updateBitmapWithBoundingBoxes(bitmap,185,465,185,145);
//                            captureFrameToFile(bitmap);
                        }


                        resultAvaliable.open();
                    }
                }));

                resultAvaliable.block();
            }
            Log.d("Vision", "RESULT " + resultLocation[0].toString());

            return resultLocation[0];
        }
        return Location.UNKNOWN;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            //  Instantiate the Vuforia engine
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

            vuforia.enableConvertFrameToBitmap();
        } catch (Exception e) {

        }
    }
}