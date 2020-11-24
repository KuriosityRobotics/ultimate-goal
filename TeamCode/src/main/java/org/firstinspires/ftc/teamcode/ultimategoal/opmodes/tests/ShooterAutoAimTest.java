package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

import java.util.ArrayList;

@TeleOp
public class ShooterAutoAimTest extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    long lastUpdateTime;

    Toggle bToggle = new Toggle();
    Toggle aToggle = new Toggle();

    private static final double SLOW_MODE_SCALE_FACTOR = 0.3;

    private boolean lastArrowMoveState = false;
    private double arrowMoveAngle = 0;

    public void runOpMode() {
        initRobot();

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            if (aToggle.isToggled(gamepad1.a)) {
                robot.shooter.isAimBotActive = !robot.shooter.isAimBotActive;
            }

            if (robot.shooter.isAimBotActive) {
                if (bToggle.isToggled(gamepad1.b)) {
                    robot.shooter.queueRingIndex();
                }
            } else {
                updateDrivetrainStates();
            }

            lastUpdateTime = SystemClock.elapsedRealtime();
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry,this);

        robot.drivetrain.weakBrake = true;
    }

    private void updateDrivetrainStates() {
        double yMovement = 0;
        double xMovement = 0;
        double turnMovement = 0;

        if (usingJoysticks()) {
            yMovement = -gamepad1.left_stick_y;
            xMovement = gamepad1.left_stick_x;
            turnMovement = gamepad1.right_stick_x;
        } else if (usingDPad()) {
            if (gamepad1.dpad_up) {
                yMovement = 1;
            } else if (gamepad1.dpad_down) {
                yMovement = -1;
            } else if (gamepad1.dpad_left) {
                turnMovement = -1;
            } else if (gamepad1.dpad_right) {
                turnMovement = 1;
            }
        }

        if (gamepad1.left_bumper) {
            if (!lastArrowMoveState) {
                arrowMoveAngle = robot.drivetrain.getCurrentHeading();
                lastArrowMoveState = true;
            }

            double r = Math.hypot(yMovement, xMovement);
            double aT = Math.atan2(yMovement, xMovement);
            double t = aT + robot.drivetrain.getCurrentHeading() - arrowMoveAngle;

            double nXMovement = r * Math.cos(t);
            double nYMovement = r * Math.sin(t);

            xMovement = nXMovement;
            yMovement = nYMovement;
        } else {
            lastArrowMoveState = false;
        }

        // TODO: use brakemode of branch AutoActions instead
        if (gamepad1.right_bumper) {
            xMovement *= SLOW_MODE_SCALE_FACTOR;
            yMovement *= SLOW_MODE_SCALE_FACTOR;
            turnMovement *= SLOW_MODE_SCALE_FACTOR;
        }

        robot.drivetrain.setMovements(xMovement, yMovement, turnMovement);
    }

    private boolean usingJoysticks() {
        return gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0;
    }

    private boolean usingDPad() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        long currentTime = SystemClock.elapsedRealtime();

        ArrayList<String> data = new ArrayList<>();
        data.add("How to use: Begin the robot at the front blue corner of the field (away from the tower goal). Use gamepad1 to drive the robot. Press 'a' to toggle aiming mode, and then 'b' to queue a shot.");
        data.add("TeleOp while loop update time: " + (currentTime - lastUpdateTime));
        lastUpdateTime = currentTime;

        return data;
    }

    public String getName() {
        return "ShooterAutoAim";
    }
}
