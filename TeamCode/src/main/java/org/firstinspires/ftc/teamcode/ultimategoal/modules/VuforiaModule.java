package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions;

import java.util.ArrayList;
import java.util.List;

public class VuforiaModule implements Module, TelemetryProvider {

    private Robot robot;
    private boolean isOn;
    private String VUFORIA_KEY = "AblAnwD/////AAABmRHXA7f65ErFhMbZmr+8xjArZA83Y+l2nal3r90Dmzl6nc0hUj+zgUCK3sF8PxkhDDJkMsSJSl05Q3U0Bjz6HeydKoGMwsvF8x2IbUto/6gbCm8WqDkvfBjzDeVL5Y3XCkczOi1F8dmNt1JkJQdX4bJokLrzEBQnQOF6mwxI22M2eSobTgyHSrZk4hl6jTXVSO9ckVtMfVjV/pryDQMnnJDFMQ/64u+uhxtnsMZKgd9UlORAMwsSL9Wwk1ixoWeUsLzZS4w/5b4GbupBTsY/teWORJo0AulqTI+rCJRhKzQcZRlG7v5jt2f3es7y0uXbxT5QrQ06tNmvhEfjAIduF5eSbPZ/3QjQnFgtRuyMW0ix";

    public double tRX;
    public double tRY;
    public double tRPhi;
    public VuforiaTrackable tracker;

    private VuforiaLocalizer vuforia = null;
    private WebcamName webcamName = null;

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTrackables targetsUltimateGoal;

    public boolean visibleTracker = false; // note: do not take data from this vuf stream if no tracker is visible




    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private final float targetHeight   = 6f;

    // Constants for perimeter targets
    private final float fullField = 144f;
    private final float halfField = 72f;
    private final float quadField  = 36f;

    private final float cornerOffsetX = -9f;
    private final float cornerOffsetY = -8.25f;





    public VuforiaModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;
        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void initModules() {

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = robot.hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        // NOTE: rather than using x,y,z, using x,y,theta to get know pose
        redAllianceTarget.setLocation(OpenGLMatrix.translation(fullField+cornerOffsetX, halfField+cornerOffsetY, (float)Math.toRadians(270)));
        blueAllianceTarget.setLocation(OpenGLMatrix.translation(cornerOffsetX, halfField+cornerOffsetY, (float)Math.toRadians(90)));

        frontWallTarget.setLocation(OpenGLMatrix.translation(halfField+cornerOffsetX, cornerOffsetY, 0f));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix.translation(quadField+cornerOffsetX, fullField+cornerOffsetY, (float)Math.toRadians(180)));
        redTowerGoalTarget.setLocation(OpenGLMatrix.translation(halfField+quadField+cornerOffsetX, fullField+cornerOffsetY, (float)Math.toRadians(180)));

        // placed here because takes a while to activate, technically could be moved to onStart() due to low dependency on vuf at beginning
        targetsUltimateGoal.activate();
        robot.telemetry.addLine("DONE INITING VUFORIA");
    }

    @Override
    public void update() {
        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix cameraFromTarget = ((VuforiaTrackableDefaultListener)trackable.getListener()).getFtcCameraFromTarget();

                if (cameraFromTarget != null) {
                    VectorF trans = cameraFromTarget.getTranslation();
                    Orientation rot = Orientation.getOrientation(cameraFromTarget, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

                    tracker = trackable;

                    //TODO: need to convert from camera coordinates to robot coordinates
                    tRX = trans.get(0)/25.4; // right now this is left right camera coords
                    tRY = trans.get(2)/25.4; // right now this is forward backward camera coords
                    tRPhi = -MathFunctions.angleWrap(rot.secondAngle); // right now this is angle in camera coords
                }
                break;
            }
        }

        if (targetVisible) {
            visibleTracker = true;
        }  else {
            visibleTracker = false;
        }
    }

    @Override
    public void onClose() {
        targetsUltimateGoal.deactivate();
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        if (tracker != null){
            data.add("Tracker Name: " + tracker.getName());
            data.add("Tracker X: " + (double)tracker.getLocation().getTranslation().get(0));
            data.add("Tracker Y: " + (double)tracker.getLocation().getTranslation().get(1));
            data.add("Tracker Phi: " + (double)Math.toDegrees(tracker.getLocation().getTranslation().get(2)));
        }
        data.add("tX: " + tRX);
        data.add("tY: " + tRY);
        data.add("rPhi: " + Math.toDegrees(tRPhi));
        return data;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "VuforiaModule";
    }
}
