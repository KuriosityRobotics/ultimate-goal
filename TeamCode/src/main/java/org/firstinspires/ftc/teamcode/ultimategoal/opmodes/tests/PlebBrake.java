package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;

@Autonomous
public class PlebBrake extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        robot.startModules();

        robot.drivetrain.setMovements(0, 0, 0);

        robot.drivetrain.brakePoint = new Point(0, 60);
//        robot.drivetrain.brakeHeading = Math.toRadians(90);

        while (opModeIsActive()) {

        }
    }
}
