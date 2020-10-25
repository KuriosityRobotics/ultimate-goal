package org.firstinspires.ftc.teamcode.ultimategoal.TestOpModes;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;

@Autonomous
public class SimpleStraightTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        PathFollow pf1 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 0),
                new Waypoint(0, 24),
                new Waypoint(0, 48)
        }, robot, "test");

        PathFollow pf2 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 48),
                new Waypoint(0, 24),
                new Waypoint(0, 0)
        }, robot, "test");

        waitForStart();

        robot.startModules();

        pf1.pathFollow(0, 1, 1, true, 0);

        sleep(5000);

        pf2.pathFollow(Math.PI, 1, 1, true, 0);

        sleep(5000);
    }
}
