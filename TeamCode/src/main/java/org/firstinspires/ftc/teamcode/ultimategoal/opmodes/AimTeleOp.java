package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.BluePowershotsAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;

@TeleOp
public class AimTeleOp extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    long lastUpdateTime = 0;
    long loopTime;

    Toggle powershotsToggle = new Toggle();

    Toggle resetCountersToggle = new Toggle();

    Toggle flyWheelToggle = new Toggle();
    Toggle nextTargetToggle = new Toggle();

    Toggle deliverRingsToggle = new Toggle();
    Toggle autoManagerToggle = new Toggle();

    Toggle queueIndexesToggle = new Toggle();
    Toggle forceIndexToggle = new Toggle();

    Toggle wobbleClawToggle = new Toggle();
    Toggle wobbleArmToggle = new Toggle();
    Toggle blockerToggle = new Toggle();

    Toggle autoTurretToggle = new Toggle();

    Toggle manualAngleAddToggle = new Toggle();
    Toggle manualAngleSubtractToggle = new Toggle();
    Toggle manualFlapAddToggle = new Toggle();
    Toggle manualFlapSubtractToggle = new Toggle();
    Toggle resetCorrectionsToggle = new Toggle();

    BluePowershotsAction bluePowershotsAction = new BluePowershotsAction();
    private boolean doPowershotsAction = false;

    Point POWERSHOT = new Point(44, 51.25);

    public void runOpMode() {
        initRobot();

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.BLOCKING;

        while (opModeIsActive()) {
            if (powershotsToggle.isToggled(gamepad1.b)) {
                doPowershotsAction = !doPowershotsAction;

                if (doPowershotsAction) {
                    bluePowershotsAction = new BluePowershotsAction();

                    robot.shooter.manualAngleFlapCorrection = 0;
                    robot.shooter.manualAngleCorrection = 0;
                } else {
                    robot.shooter.flywheelOn = false;
                    robot.drivetrain.setBrakePosition(robot.drivetrain.getCurrentPosition());
                }
            }

            if (resetCountersToggle.isToggled(gamepad2.right_bumper)) {
                robot.ringManager.resetRingCounters();
            }

            if (doPowershotsAction) {
                robot.shooter.flywheelOn = true;

                if(robot.drivetrain.distanceToPoint(POWERSHOT) > 5) {
                    robot.drivetrain.setMovementsTowardsPoint(POWERSHOT, 1, 1, 0, true, 0);
                } else{
                    if (robot.drivetrain.distanceToPoint(POWERSHOT) < 1) {
                        doPowershotsAction = !bluePowershotsAction.executeAction(robot);
                    }
                    robot.drivetrain.setBrakePosition(POWERSHOT);
                }
            } else {
                robot.shooter.lockTarget = true;

                updateTurretStates();
                updateRingDeliverySystemStates();
                updateManualCorrections();

                updateDrivetrainStates();

                updateWobbleStates();

                updateIntakeStates();
            }

            long currentTime = SystemClock.elapsedRealtime();
            loopTime = currentTime - lastUpdateTime;
            lastUpdateTime = currentTime;
        }
    }

    private void updateTurretStates() {
        if (nextTargetToggle.isToggled(gamepad1.y)) {
            robot.shooter.nextTarget();
        }

        if (flyWheelToggle.isToggled(gamepad1.a)) {
            robot.shooter.flywheelOn = !robot.shooter.flywheelOn;

            if (robot.shooter.flywheelOn) {
                robot.shooter.target = BLUE_HIGH;
            }
        }

        if (robot.shooter.flywheelOn) {
            if (queueIndexesToggle.isToggled(gamepad1.right_trigger)) {
                robot.shooter.queueIndex(3);
            }
        }

        if (forceIndexToggle.isToggled(gamepad1.left_bumper)) {
            robot.shooter.forceIndex();
            robot.shooter.queueIndex();
        }

//        if (Math.abs(gamepad2.left_stick_x) > 0.1 || robot.shooter.manualTurret) {
//            robot.shooter.manualTurret = true;
//            robot.shooter.manualTurretPower = gamepad2.left_stick_x;
//        }
//
//        if (autoTurretToggle.isToggled(gamepad2.left_bumper)) {
//            robot.shooter.manualTurret = false;
//            robot.shooter.manualTurretPower = 0;
//        }
    }

    private void updateRingDeliverySystemStates() {
        if (deliverRingsToggle.isToggled(gamepad2.a)) {
            robot.shooter.deliverRings();
        }

        if (autoManagerToggle.isToggled(gamepad1.x)) {
            robot.ringManager.autoRaise = !robot.ringManager.autoRaise;
            robot.ringManager.autoShootRings = !robot.ringManager.autoShootRings;
        }
    }

    private void updateManualCorrections() {
        if (manualAngleAddToggle.isToggled(gamepad2.dpad_right)) {
            robot.shooter.manualAngleCorrection += Math.toRadians(2);
        }
        if (manualAngleSubtractToggle.isToggled(gamepad2.dpad_left)) {
            robot.shooter.manualAngleCorrection -= Math.toRadians(2);
        }
        if (manualFlapAddToggle.isToggled(gamepad2.dpad_up)) {
            robot.shooter.manualAngleFlapCorrection += 0.003;
        }
        if (manualFlapSubtractToggle.isToggled(gamepad2.dpad_down)) {
            robot.shooter.manualAngleFlapCorrection -= 0.003;
        }
        if (resetCorrectionsToggle.isToggled(gamepad2.right_trigger)) {
            robot.shooter.manualAngleFlapCorrection = 0;
            robot.shooter.manualAngleCorrection = 0;
        }
    }

    private void updateWobbleStates() {
        if (wobbleClawToggle.isToggled(gamepad2.y)) {
            robot.wobbleModule.isClawClamped = !robot.wobbleModule.isClawClamped;
        }

        if (wobbleArmToggle.isToggled(gamepad2.b)) {
            robot.wobbleModule.nextArmPosition();
        }
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = gamepad2.right_stick_y * 2;

        if (blockerToggle.isToggled(gamepad2.x)) {
            robot.intakeModule.blockerPosition = robot.intakeModule.blockerPosition.next();
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this, BlueAuto.PARK, false);

        robot.drivetrain.weakBrake = true;

        robot.shooter.lockTarget = true;
        robot.shooter.manualTurret = false;
    }

    boolean wasNoPower = false;
    long lastPowerTime = 0;

    private void updateDrivetrainStates() {
        double yMovement;
        double xMovement;
        double turnMovement;

        yMovement = -gamepad1.left_stick_y;
        xMovement = gamepad1.left_stick_x;
        turnMovement = gamepad1.right_stick_x;

        if (gamepad1.dpad_down){
            robot.drivetrain.setPosition(26.0,72.0,0.0);
        }

        robot.drivetrain.isSlowMode = gamepad1.right_bumper;

        robot.drivetrain.setMovements(xMovement, yMovement, turnMovement);

        boolean isNoPower = Math.abs(xMovement) > 0 && Math.abs(yMovement) > 0 && Math.abs(turnMovement) > 0;
        if (isNoPower && !wasNoPower) {
            lastPowerTime = SystemClock.elapsedRealtime();
        }
        wasNoPower = isNoPower;

        boolean robotStopped = Math.hypot(robot.drivetrain.getOdometryXVel(), robot.drivetrain.getOdometryYVel()) < 0.5
                && Math.abs(robot.drivetrain.getOdometryAngleVel()) < Math.toRadians(0.1);
        if (isNoPower && SystemClock.elapsedRealtime() > lastPowerTime + 750 && robotStopped) {
            robot.drivetrain.weakBrake = false;
        } else {
            robot.drivetrain.weakBrake = true;
        }
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("TeleOp while loop update time: " + loopTime);
        return data;
    }

    public String getName() {
        return "ShooterAutoAim";
    }
}
