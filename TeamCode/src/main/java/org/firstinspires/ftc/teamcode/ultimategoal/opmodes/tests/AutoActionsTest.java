package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;

@Autonomous
public class AutoActionsTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, new Pose2d(), false);

        PathFollow pf1 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 0),
                new Waypoint(0, 24),
                new Waypoint(0, 60)
        }, robot, "test");

        waitForStart();

        robot.startModules();

        sleep(1000);

        pf1.followPath(0, 1, 1, false, Math.toRadians(90));

        while (opModeIsActive()) {

        }
    }
}
