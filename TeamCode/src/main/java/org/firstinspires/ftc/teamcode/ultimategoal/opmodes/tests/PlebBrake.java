package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

@Config
@TeleOp
public class PlebBrake extends LinearOpMode {
    Robot robot;

    public static double ANGLE = 90;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, new Pose2d(), false);

        waitForStart();

        robot.startModules();

        robot.drivetrain.weakBrake = false;
        robot.drivetrain.setMovements(0, 0, 0);

//        robot.drivetrain.brakePoint = new Point(0, 60);

        while (opModeIsActive()) {
            robot.drivetrain.brakeHeading = Math.toRadians(ANGLE);
        }
    }
}
