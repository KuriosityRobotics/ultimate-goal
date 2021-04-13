package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

@TeleOp
public class TurretTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        robot.drivetrain.brake = false;

        robot.shooter.lockTarget = false;

        //robot.shooter.setTurretTargetangle(Math.toRadians(180));

        robot.startModules();

        while (opModeIsActive()) {

        }
    }
}
