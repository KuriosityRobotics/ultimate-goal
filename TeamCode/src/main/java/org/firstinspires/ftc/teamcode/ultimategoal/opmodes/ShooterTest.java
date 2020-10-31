package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

import java.util.ArrayList;

@TeleOp
public class ShooterTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    Toggle a = new Toggle();

    // States
    double flapPosition = 0.74; // In degrees
    double flyWheelSpeed = 1550; // In ticks per second

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);
        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            flapPosition -= gamepad1.right_stick_y * 0.000001;
            flyWheelSpeed -= gamepad1.left_stick_y * 0.01;

            if (flapPosition > 1) {
                flapPosition = 1;
            } else if (flapPosition < 0) {
                flapPosition = 0;
            }

            robot.shooter.setFlapPosition(flapPosition);
            robot.shooter.setFlyWheelSpeed(flyWheelSpeed);

            if (a.isToggled(gamepad1.a)) {
                robot.shooter.queueRingIndex();
            }
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Fly wheel target speed: " + flyWheelSpeed);
        data.add("Shooter flap target angle: " + flapPosition);

        return data;
    }

    public String getName() {
        return "ShooterTest";
    }
}
