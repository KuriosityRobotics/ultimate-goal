package org.firstinspires.ftc.teamcode.ultimategoal;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Module;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.ShooterModule;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.VelocityModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Drivetrain;
import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDump;
import org.firstinspires.ftc.teamcode.ultimategoal.util.ModuleExecutor;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryDump;

public class Robot {
    // All modules in the robot (remember to update initModules() and updateModules() when adding)
    public Drivetrain drivetrain;
    public VelocityModule velocityModule;
    public ShooterModule shooterModule;

    public long currentTimeMilli;

    public HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    public TelemetryDump telemetryDump;
    public FileDump fileDump;

    // New thread that updates modules
    ModuleExecutor moduleExecutor;

    // Array that all modules will be loaded into for easier access
    private Module[] modules;

    // REV Hubs
    private LynxModule revHub1;
    private LynxModule revHub2;

    public final boolean WILL_FILE_DUMP = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        this.telemetryDump = new TelemetryDump(telemetry);
        fileDump = new FileDump();

        initHubs();
        initModules();
    }

    public void update() {
        refreshHubData();

        currentTimeMilli = SystemClock.elapsedRealtime();

        for (Module module : modules) {
            if (module.isOn()) {
                try {
                    module.update();
                } catch (Exception e) {
                    Log.d("Module", "Module couldn't update: " + module.getName());
                }
            }
        }
        telemetryDump.update();

        if(isStopRequested()) {
            this.cleanUp();
        }
    }

    private void cleanUp() {
        this.fileDump.writeFilesToDevice();
        try {
            this.moduleExecutor.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void initModules() {
        // Add individual modules into the array here
        this.drivetrain = new Drivetrain(this, true);
        this.velocityModule = new VelocityModule(this, true);
        this.shooterModule = new ShooterModule(this, true);

        this.modules = new Module[]{
                this.drivetrain, this.velocityModule, this.shooterModule
        };

        // Initialize modules
        for (Module module : modules) {
            module.init();
        }

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this);
    }

    /**
     * Starts running the loop that updates modules
     */
    public void startModules() {
        moduleExecutor.start();
    }

    private void initHubs() {
        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
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

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public boolean isStopRequested() {
        return linearOpMode.isStopRequested();
    }

    private void ಢ_ಢ() {
        throw new Error("ರ_ರ plz dont rely on finalize to cleanup");
    }
    public void finalize() {
        this.cleanUp();
        ಢ_ಢ();
    }
}
