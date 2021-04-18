package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.IntakeModule;
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
    Toggle g1LB = new Toggle();

    Toggle g2x = new Toggle();
    Toggle g2a = new Toggle();
    Toggle g2b = new Toggle();
    Toggle g2y = new Toggle();
    Toggle g2DR = new Toggle();
    Toggle g2DL = new Toggle();
    Toggle g2DU = new Toggle();
    Toggle g2DD = new Toggle();
    Toggle g2LB = new Toggle();
    Toggle g2RB = new Toggle();

    BluePowershotsAction bluePowershotsAction = new BluePowershotsAction();
    private boolean doPowershotsAction = false;

    public void runOpMode() {
        initRobot();

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.startModules();

        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.BLOCKING;

        while (opModeIsActive()) {
            if (g1b.isToggled(gamepad1.b)) {
                doPowershotsAction = !doPowershotsAction;

                if (doPowershotsAction) {
                    bluePowershotsAction = new BluePowershotsAction();
                } else {
                    robot.shooter.flywheelOn = false;
                }
            }

            if(g2RB.isToggled(gamepad2.right_bumper)){
                robot.ringManager.resetRingCounters();
            }

            if (doPowershotsAction) {
                robot.shooter.manualAngleFlapCorrection = 0;
                robot.shooter.manualAngleCorrection = 0;

                doPowershotsAction = !bluePowershotsAction.executeAction(robot);
            } else {
                updateShooterStates();

                updateDrivetrainStates();

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
            robot.shooter.flywheelOn = !robot.shooter.flywheelOn;

            if (robot.shooter.flywheelOn) {
                robot.shooter.target = BLUE_HIGH;
            }
        }

        if (g2a.isToggled(gamepad2.a)) {
            robot.shooter.deliverRings();
        }

        if (robot.shooter.flywheelOn) {
            if (g1RT.isToggled(gamepad1.right_trigger)) {
                robot.shooter.queueIndex(3);
            }
        }

        if (g1x.isToggled(gamepad1.x)) {
            robot.shooter.lockTarget = !robot.shooter.lockTarget;
        }

        if (g1LB.isToggled(gamepad1.left_bumper)) {
            robot.shooter.forceIndex();
            robot.shooter.queueIndex();
        }

        if (g2DR.isToggled(gamepad2.dpad_right)) {
            robot.shooter.manualAngleCorrection += Math.toRadians(2);
        }
        if (g2DL.isToggled(gamepad2.dpad_left)) {
            robot.shooter.manualAngleCorrection -= Math.toRadians(2);
        }
        if (g2DU.isToggled(gamepad2.dpad_up)) {
            robot.shooter.manualAngleFlapCorrection += 0.003;
        }
        if (g2DD.isToggled(gamepad2.dpad_down)) {
            robot.shooter.manualAngleFlapCorrection -= 0.003;
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

        if (g2b.isToggled(gamepad2.b)) {
            robot.wobbleModule.nextArmPosition();
        }
    }

    private void updateIntakeStates() {
        robot.intakeModule.intakePower = gamepad2.left_stick_y * 2;

        if (g2y.isToggled(gamepad2.y)) {
            robot.intakeModule.blockerPosition = robot.intakeModule.blockerPosition.next();
        }
    }

    private void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this, BlueAuto.PARK);

        robot.drivetrain.weakBrake = true;

        robot.shooter.lockTarget = true;
    }

    private void updateDrivetrainStates() {
        double yMovement;
        double xMovement;
        double turnMovement;

        yMovement = -gamepad1.left_stick_y;
        xMovement = gamepad1.left_stick_x;
        turnMovement = gamepad1.right_stick_x;

        if (gamepad1.dpad_left) {
            robot.drivetrain.setPosition(0, robot.drivetrain.getCurrentPosition().y, 0);
        }
        if (gamepad1.dpad_down) {
            robot.drivetrain.setPosition(robot.drivetrain.getCurrentPosition().x, 0, 0);
        }
        if (gamepad1.dpad_up) {
            robot.drivetrain.setPosition(0, 0, 0);
        }

        if (gamepad1.left_bumper) {
            robot.ringManager.setDistanceSensorPasses(6);
        }

        robot.drivetrain.isSlowMode = gamepad1.right_bumper;

        robot.drivetrain.setMovements(xMovement, yMovement, turnMovement);
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
