package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PIDController;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

import java.util.ArrayList;

@TeleOp
public class PIDTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    long lastUpdateTime;

    public void runOpMode() {
        robot.telemetryDump.registerProvider(this);
        initRobot();
        PIDController pidController = new PIDController(0.01, 0.0000001, 0, robot);
        waitForStart();
        robot.startModules();

        while (opModeIsActive()) {
            long initialTime = SystemClock.elapsedRealtime();

            Point robotPosition = robot.drivetrain.getCurrentPosition();

            pidController.calculatePID(robotPosition.y, 56);
            robot.drivetrain.setMovements(0, pidController.scale, 0);

            lastUpdateTime = SystemClock.elapsedRealtime() - initialTime;
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Loop time: " + lastUpdateTime);
        return data;
    }

    public String getName() {
        return "PIDTest";
    }
}

