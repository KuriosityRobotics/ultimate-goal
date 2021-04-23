package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

//@Disabled
@Config
@TeleOp
public class TurretTest extends LinearOpMode {
    Robot robot;

    public static double TURRET_HEADING = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this, new Pose2d(), false);

        waitForStart();

        robot.drivetrain.brake = false;

        robot.shooter.lockTarget = true;

        robot.startModules();

        while (opModeIsActive()) {
            robot.shooter.setTurretTargetHeading(TURRET_HEADING);
        }
    }
}
