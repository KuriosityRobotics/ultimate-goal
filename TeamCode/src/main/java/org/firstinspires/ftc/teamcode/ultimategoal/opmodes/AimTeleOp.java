package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.HopperModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

import java.util.ArrayList;

@TeleOp
public class AimTeleOp extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    long lastUpdateTime = 0;
    long loopTime;

    Toggle g1x = new Toggle();
    Toggle g1a = new Toggle();
    Toggle g1b = new Toggle();
    Toggle g2x = new Toggle();

    private static final double SLOW_MODE_SCALE_FACTOR = 0.3;

    private boolean lastArrowMoveState = false;
    private double arrowMoveAngle = 0;

    public void runOpMode() {
        initRobot();

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            updateShooterStates();

            if (!robot.shooter.isAimBotActive) {
                updateDrivetrainStates();
                robot.shooter.setHopperPosition(HopperModule.HopperPosition.LOWERED);
            }

            updateWobbleStates();

            updateIntakeStates();

            long currentTime = SystemClock.elapsedRealtime();
            loopTime = currentTime - lastUpdateTime;
            lastUpdateTime = currentTime;
        }
    }

    private void updateShooterStates() {
        if (g1a.isToggled(gamepad1.a)) {
            robot.shooter.isAimBotActive = !robot.shooter.isAimBotActive;
        }

        if (robot.shooter.isAimBotActive) {
            if (g1b.isToggled(gamepad1.b)) {
                robot.shooter.queueIndexThreeRings();
            }
        } else {
            if (g2x.isToggled(gamepad2.x)) {
                if (robot.shooter.getFlyWheelTargetSpeed() > 0) {
                    robot.shooter.setFlyWheelSpeed(0);
                } else {
                    robot.shooter.setFlyWheelSpeed(robot.FLY_WHEEL_SPEED);
                }
            }
        }
    }

    private void updateWobbleStates() {
        if (g1x.isToggled(gamepad1.x)) {
            robot.wobbleModule.isClawClamped = !robot.wobbleModule.isClawClamped;
        }

        robot.wobbleModule.wobbleTargetPosition -= gamepad2.right_stick_y;
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = gamepad2.left_stick_y * 2;
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

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
        ArrayList<String> data = new ArrayList<>();
        data.add("How to use: Begin the robot at the front blue corner of the field (away from the tower goal). Use gamepad1 to drive the robot. Press 'a' to toggle aiming mode, and then 'b' to queue a shot.");
        data.add("TeleOp while loop update time: " + loopTime);
        return data;
    }

    public String getName() {
        return "ShooterAutoAim";
    }
}
