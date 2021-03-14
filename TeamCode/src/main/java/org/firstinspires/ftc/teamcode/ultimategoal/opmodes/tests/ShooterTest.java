package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.HopperModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

import java.util.ArrayList;

@Disabled
@TeleOp
public class ShooterTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    Toggle a = new Toggle();
    Toggle b = new Toggle();
    Toggle x = new Toggle();

    // States
    //    final double FLAP_MAX = 0.71;
    double flapPosition = 0.65; // In degrees
    double flyWheelSpeed = 1300; // In ticks per second

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);
        robot.drivetrain.zeroPowerBrake = false;

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            robot.shooter.setFlyWheelTargetSpeed(flyWheelSpeed);

            if (gamepad1.x) {
                flyWheelSpeed = 1260;
            } else if (gamepad1.y) {
                flyWheelSpeed = 1320;
            } else if (gamepad1.b) {
                flyWheelSpeed = 1550;
            }

            robot.shooter.setHopperPosition(HopperModule.HopperPosition.RAISED);

            flapPosition -= gamepad1.right_stick_y * 0.0000001;
            //            flyWheelSpeed -= gamepad1.left_stick_y * 0.001;

            if (flapPosition > 1) {
                flapPosition = 1;
            } else if (flapPosition < 0) {
                flapPosition = 0;
            }

            robot.shooter.setFlapPosition(flapPosition);

            //            robot.shooter.target = Target.Blue.BLUE_POWERSHOT1;
            robot.shooter.isFlyWheelOn = true;

            //            if (x.isToggled(gamepad1.y))
            //                flapPosition /= 2;

            if (a.isToggled(gamepad1.a)) {
                robot.shooter.requestRingIndex();
            }
            //            if (b.isToggled(gamepad1.b))
            //                robot.shooter.nextTarget();

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
