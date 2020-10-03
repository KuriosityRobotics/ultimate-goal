package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.ToggleButton;

import java.util.ArrayList;

@TeleOp
public class ShooterTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    ToggleButton a = new ToggleButton();

    double flapAngle = 90; // In degrees
    double flyWheelSpeed = 1000; // In degrees per second

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            flapAngle -= gamepad1.right_stick_y * 0.001;
            flyWheelSpeed -= gamepad1.left_stick_y * 0.005;

            if (flapAngle > 180) {
                flapAngle = 180;
            } else if (flapAngle < 0) {
                flapAngle = 0;
            }

            robot.shooterModule.shooterFlapAngle = flapAngle;
            robot.shooterModule.flyWheelTargetSpeed = flyWheelSpeed;

            if (a.isToggled(gamepad1.a)) {
                robot.shooterModule.indexRing = true;
            }
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();

        data.add("Fly wheel speed: " + flyWheelSpeed);
        data.add("Shooter flap angle: " + flapAngle);

        return data;
    }

    public String getName() {
        return "ShooterTest";
    }
}
