package org.firstinspires.ftc.teamcode.ultimategoal;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Module;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.ModuleCollection;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDump;
import org.firstinspires.ftc.teamcode.ultimategoal.util.ModuleExecutor;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryDump;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.ActionExecutor;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import static org.firstinspires.ftc.teamcode.ultimategoal.modules.OdometryModule.slamra;

public class Robot extends ModuleCollection {
    // All modules in the robot (remember to update initModules() and updateModules() when adding)
    public Drivetrain drivetrain;
    public Shooter shooter;

    public IntakeModule intakeModule;
    public WobbleModule wobbleModule;
//    public VisionModule visionModule;

    private long currentTimeMilli;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private LinearOpMode linearOpMode;

    public TelemetryDump telemetryDump;
    public FileDump fileDump;

    // New thread that updates modules
    ModuleExecutor moduleExecutor;

    // Action executor
    public ActionExecutor actionExecutor;

    // REV Hubs
    private LynxModule revHub1;
    private LynxModule revHub2;

    // Constants
    public final static boolean WILL_FILE_DUMP = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this(hardwareMap, telemetry, linearOpMode, new Point(0, 0));
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode, Point startingPosition) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        this.telemetryDump = new TelemetryDump(telemetry);
        fileDump = new FileDump();

        initHubs();
        initialize(startingPosition);

        actionExecutor = new ActionExecutor(this);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, OpMode linearOpMode) {
        this(hardwareMap, telemetry, linearOpMode, new Point(0, 0));
        linearOpMode.internalPostLoop();
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, OpMode linearOpMode, Point startingPosition) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.telemetryDump = new TelemetryDump(telemetry);
        fileDump = new FileDump();

        initHubs();
        initialize(startingPosition);

        actionExecutor = new ActionExecutor(this);
    }

    public void update() {
        refreshHubData();

        currentTimeMilli = SystemClock.elapsedRealtime();

        for (Module module : modules) {
            if (module.isOn()) {
                try {
                    module.update();
                } catch (Exception e) {
                    e.printStackTrace();
                    Log.d("Module", "Module couldn't update: " + module.getName());
                }
            }
        }
        telemetryDump.update();

        if (isStopRequested()) {
            this.cleanUp();
        }
    }

    private void cleanUp() {
        this.fileDump.writeFilesToDevice();
        try {
            slamra.stop();
            this.moduleExecutor.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void initialize(Point startingPosition) {
        // Add individual modules into the array here
        this.drivetrain = new Drivetrain(this, true, startingPosition);
        this.shooter = new Shooter(this, true);

        this.intakeModule = new IntakeModule(this, true);
        this.wobbleModule = new WobbleModule(this, true);
//        this.visionModule = new VisionModule(this, true);

        this.modules = new Module[]{
                this.drivetrain, this.intakeModule, this.wobbleModule, this.shooter
        };

        // Initialize modules
        initModules(); // Initial init
        while (!initAsync()) {
        } // Cycle init

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this);

        // Tick telemetryDump once to get the lines to show up
        telemetryDump.update();
    }

    /**
     * Starts running the loop that updates modules
     */
    public void startModules() {
        onStart();
        moduleExecutor.start();
    }

    private void initHubs() {
        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 173");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
            linearOpMode.stop();
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void refreshHubData() {
        revHub1.getBulkData();
        revHub2.getBulkData();
    }

    public DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor with name " + name + " could not be found. Exception: " + exception);
        }
    }

    public Servo getServo(String name) {
        try {
            return hardwareMap.servo.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Servo with name " + name + " could not be found. Exception: " + exception);
        }
    }

    public synchronized void setLedColor(int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            throw new IllegalArgumentException();
        }

        setLedColor((byte) r, (byte) g, (byte) b);
    }

    /***
     * Set the color of the Expansion Hub's RGB status LED
     *
     * @param r red value
     * @param g green value
     * @param b blue value
     */
    public synchronized void setLedColor(byte r, byte g, byte b) {
        LynxSetModuleLEDColorCommand colorCommand = new LynxSetModuleLEDColorCommand(revHub1, r, g, b);
        LynxSetModuleLEDColorCommand colorCommand2 = new LynxSetModuleLEDColorCommand(revHub2, r, g, b);

        try {
            colorCommand.send();
            colorCommand2.send();
        } catch (InterruptedException | LynxNackException e) {
            Log.d("EXCEPTION COLOR", e.toString());
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isStopRequested() {
        return linearOpMode.isStopRequested();
    }

    public long getCurrentTimeMilli() {
        return currentTimeMilli;
    }

    public void opModeSleep(long milliseconds) {
        linearOpMode.sleep(milliseconds);
    }

    private void ಢ_ಢ() {
        throw new Error("ರ_ರ plz dont rely on finalize to cleanup");
    }

    public void finalize() {
        this.cleanUp();
        ಢ_ಢ();
    }

    @Override
    public boolean isOn() {
        return true;
    }

    @Override
    public String getName() {
        return "Robot";
    }
}
