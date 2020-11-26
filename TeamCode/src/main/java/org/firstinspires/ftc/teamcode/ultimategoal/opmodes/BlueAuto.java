package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.RingStackLocator;

public class BlueAuto extends LinearOpMode {
    Robot robot;

    final Point STARTING = new Point(23 + 24 + 5, 9);

    final Point POWERSHOT = new Point(STARTING.x, 23.5 * 3);

    final Point TARGET_A_DROPOFF_FIRST = new Point(12, 80);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2, 94.25 + 12);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23.5, 117.75 + 12);

    Point firstWobbleDropOff;

    final Point STACK = new Point(34, 47);

    final Point SECOND_WOBBLE = new Point(33, 33);

    final Point TARGET_A_DROPOFF_SECOND = new Point(18, 88);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12, 94.25 + 4);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18, 117.75 + 4);

    Point secondWobbleDropoff;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, STARTING);

        telemetry.addLine("This is the Blue Autonomous Op mode.");
        telemetry.addLine("Align the left inside fender of the robot with the right of the right blue starting line.");
        telemetry.addLine("Load a wobble goal and three rings.");
        telemetry.update();

        waitForStart();

        // TODO FIND WOBBLE

        PathFollow startToPowershot = new PathFollow(new Waypoint[]{
                new Waypoint(STARTING),
                new Waypoint(POWERSHOT)
        }, robot, "Start to powershot");

        startToPowershot.pathFollow(0, 1, 1, false, 0);

        PathFollow powershotToWobble = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT),
                new Waypoint(firstWobbleDropOff)
        }, robot, "Powershot to wobble");
    }
}
