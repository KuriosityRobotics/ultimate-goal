package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.BluePowershotsAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleArmAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleClawAction;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.RingStackLocator;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;

public class BlueAuto extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    final Point STARTING = new Point(23 + 24 + 5, 9);

    final Point POWERSHOT = new Point(STARTING.x, 23.5 * 3);

    RingStackLocator.TargetZone measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(12, 80);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2, 94.25 + 12);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23.5, 117.75 + 12);

    Point firstWobbleDropOff;

    final Point STACK = new Point(34, 47);

    final Point SECOND_WOBBLE = new Point(33, 33);

    final double SHOOT_RING_Y = 47 + 12;

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

        measuredZone = RingStackLocator.TargetZone.TARGET_ZONE_B;
        switch (measuredZone) {
            case TARGET_ZONE_UNKNOWN:
            case TARGET_ZONE_A:
                firstWobbleDropOff = TARGET_A_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_A_DROPOFF_SECOND;
                break;
            case TARGET_ZONE_B:
                firstWobbleDropOff = TARGET_B_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_B_DROPOFF_SECOND;
                break;
            case TARGET_ZONE_C:
                firstWobbleDropOff = TARGET_C_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_C_DROPOFF_SECOND;
                break;
        }

        PathFollow startToPowershot = new PathFollow(new Waypoint[]{
                new Waypoint(STARTING),
                new Waypoint(POWERSHOT, new BluePowershotsAction())
        }, robot, "Start to powershot");

        PathFollow powershotToWobbleDropoff = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT, new WobbleArmAction(true)),
                new Waypoint(firstWobbleDropOff, new WobbleClawAction(false))
        }, robot, "Powershot to first wobble dropoff");

        ArrayList<Action> secondWobbleActions = new ArrayList<>();
        secondWobbleActions.add(new IntakeAction(false));
        secondWobbleActions.add(new WobbleClawAction(true));
        PathFollow wobbleDropoffToSecondWobble = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff),
                new Waypoint(STACK, new IntakeAction(true)),
                new Waypoint(SECOND_WOBBLE, secondWobbleActions)
        }, robot, "First wobble drop off to second wobble");

        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE),
                new Waypoint(secondWobbleDropoff.x, SHOOT_RING_Y, new ShootAction(BLUE_HIGH)),
                new Waypoint(secondWobbleDropoff, new WobbleClawAction(false))
        }, robot, "Second wobble to second wobble dropoff");

        PathFollow secondWobbleDropOffToPark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff.x, 80)
        }, robot, "Second wobble drop off to park");

        startToPowershot.pathFollow(0, 1, 1, false, 0);
        sleep(1000);

        powershotToWobbleDropoff.pathFollow(0, 1, 1, true, Math.toRadians(-90));
        sleep(1000);

        wobbleDropoffToSecondWobble.pathFollow(0, 1, 1, true, Math.toRadians(180));
        sleep(1000);

        secondWobbleToSecondWobbleDropOff.pathFollow(0, 1, 1, true, 0);
        sleep(1000);

        secondWobbleDropOffToPark.pathFollow(0, 1, 1, false, 0);
        sleep(5000);
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Measured target: " + measuredZone.toString());
        return data;
    }

    @Override
    public String getName() {
        return "BlueAuto";
    }
}
