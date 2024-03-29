package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;

@Disabled
@Autonomous
public class AutoPathTest extends LinearOpMode {

    public Robot robot;

    PathFollow pf1;
    PathFollow pf2;

    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, this, new Pose2d(), false);

        pf1 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 0),
                new Waypoint(24, 24),
                new Waypoint(24, 48)
        }, robot, "test1"
        );

        pf2 = new PathFollow(new Waypoint[]{
                new Waypoint(24, 48),
                new Waypoint(24, 24),
                new Waypoint(0, 0)
        }, robot, "test2"
        );

        waitForStart();

        robot.startModules();

        pf1.followPath(0, 0.8, 0.8, true, 0);

        sleep(2500);

        pf2.followPath(Math.PI, 0.8, 0.8, true, 0);

        sleep(10000);
    }
}

