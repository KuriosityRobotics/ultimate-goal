package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.BluePowershotsAction;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;

@TeleOp
public class AimTeleOp extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    long lastUpdateTime = 0;
    long loopTime;

    Toggle g1a = new Toggle();
    Toggle g1b = new Toggle();
    Toggle g1x = new Toggle();
    Toggle g1y = new Toggle();
    Toggle g1RT = new Toggle();
    Toggle g2x = new Toggle();
    Toggle g2a = new Toggle();
    Toggle g1LB = new Toggle();
    Toggle g2DR = new Toggle();
    Toggle g2DL = new Toggle();
    Toggle g2DU = new Toggle();
    Toggle g2DD = new Toggle();
    Toggle g2LB = new Toggle();

    BluePowershotsAction bluePowershotsAction = new BluePowershotsAction();
    private boolean doPowershotsAction = false;

    private static final double SLOW_MODE_SCALE_FACTOR = 0.3;

    private boolean lastArrowMoveState = false;
    private double arrowMoveAngle = 0;

    public void runOpMode() {
        initRobot();

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        while (opModeIsActive()) {
            if (g1b.isToggled(gamepad1.b)) {
                doPowershotsAction = !doPowershotsAction;

                if (doPowershotsAction) {
                    bluePowershotsAction = new BluePowershotsAction();
                } else {
                    robot.shooter.isAimBotActive = false;
                }
            }

            if (doPowershotsAction) {
                doPowershotsAction = !bluePowershotsAction.executeAction(robot);
            } else {
                updateShooterStates();

                if (!robot.shooter.isAimBotActive) {
                    updateDrivetrainStates();
                }

                updateWobbleStates();

                updateIntakeStates();
            }

            long currentTime = SystemClock.elapsedRealtime();
            loopTime = currentTime - lastUpdateTime;
            lastUpdateTime = currentTime;
        }
    }

    private void updateShooterStates() {
        if (g1y.isToggled(gamepad1.y)) {
            robot.shooter.nextTarget();
        }

        if (g1a.isToggled(gamepad1.a)) {
            robot.shooter.isAimBotActive = !robot.shooter.isAimBotActive;

            if (robot.shooter.isAimBotActive) {
                robot.shooter.target = BLUE_HIGH;
            }
        }

        if (robot.shooter.isAimBotActive) {
            if (g1RT.isToggled(gamepad1.right_trigger)) {
                robot.shooter.queueIndexThreeRings();
            }
        } else {
            if (g1x.isToggled(gamepad1.x)) {
                if (robot.shooter.getFlyWheelTargetSpeed() > 0) {
                    robot.shooter.setFlyWheelSpeed(0);
                } else {
                    robot.shooter.setFlyWheelSpeed(robot.FLY_WHEEL_SPEED);
                }
            }
        }

        if (g1LB.isToggled(gamepad1.left_bumper)) {
            robot.shooter.queueIndex();
        }
        if (g2DR.isToggled(gamepad2.dpad_right)) {
            robot.shooter.manualAngleCorrection += Math.toRadians(2);
            robot.shooter.resetAiming();
        }
        if (g2DL.isToggled(gamepad2.dpad_left)) {
            robot.shooter.manualAngleCorrection -= Math.toRadians(2);
            robot.shooter.resetAiming();
        }
        if (g2DU.isToggled(gamepad2.dpad_up)) {
            robot.shooter.manualAngleFlapCorrection += 0.003;
            robot.shooter.resetAiming();
        }
        if (g2DD.isToggled(gamepad2.dpad_down)) {
            robot.shooter.manualAngleFlapCorrection -= 0.003;
            robot.shooter.resetAiming();
        }
        if (g2LB.isToggled(gamepad2.left_bumper)) {
            robot.shooter.manualAngleFlapCorrection = 0;
            robot.shooter.manualAngleCorrection = 0;
        }
    }

    private void updateWobbleStates() {
        if (g2x.isToggled(gamepad2.x)) {
            robot.wobbleModule.isClawClamped = !robot.wobbleModule.isClawClamped;
        }

        if (g2a.isToggled(gamepad2.a)) {
            robot.wobbleModule.nextArmPosition();
        }
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = gamepad2.left_stick_y * 2;
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this, BlueAuto.PARK);

        robot.drivetrain.weakBrake = true;
    }

    private void updateDrivetrainStates() {
        double yMovement = 0;
        double xMovement = 0;
        double turnMovement = 0;

        yMovement = -gamepad1.left_stick_y;
        xMovement = gamepad1.left_stick_x;
        turnMovement = gamepad1.right_stick_x;

        if (gamepad1.dpad_left) {
            robot.drivetrain.setPosition(0, robot.drivetrain.getCurrentPosition().y, 0);
        }
        if (gamepad1.dpad_down) {
            robot.drivetrain.setPosition(robot.drivetrain.getCurrentPosition().x, 0, 0);
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

        robot.drivetrain.isSlowMode = gamepad1.right_bumper;

        robot.drivetrain.setMovements(xMovement, yMovement, turnMovement);
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
