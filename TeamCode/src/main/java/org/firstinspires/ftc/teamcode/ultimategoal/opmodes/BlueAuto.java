package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.BluePowershotsAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.FlywheelAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.SlowModeAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleArmAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleClawAction;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.Vision;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;

@Autonomous
public class BlueAuto extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    Vision vision;

    final Point STARTING = new Point(48 - 9, 0);

    final Point POWERSHOT = new Point(STARTING.x, 23.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23 + 12, 80 - (16.5 / 2));
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 2, 94.25 + 12 - (16.5 / 2));
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 + 12, 117.75 + 12 - (16.5 / 2));

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 9, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(22.75 - 17, 22.75 - 9 + (16.5 / 2) + 7);

    final double SHOOT_RING_Y = 47 - 15;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 9, 88 - 9);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 12);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18 - 9, 117.75 - 12);

    Point secondWobbleDropoff;

    final double PARK_Y = 80 - (16.5 / 2);

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, STARTING);
        vision = new Vision(this);

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        measuredZone = vision.runDetection();
        switch (measuredZone) {
            case UNKNOWN:
            case A:
                firstWobbleDropOff = TARGET_A_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_A_DROPOFF_SECOND;
                break;
            case B:
                firstWobbleDropOff = TARGET_B_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_B_DROPOFF_SECOND;
                break;
            case C:
                firstWobbleDropOff = TARGET_C_DROPOFF_FIRST;
                secondWobbleDropoff = TARGET_C_DROPOFF_SECOND;
                break;
        }

        robot.startModules();

        PathFollow startToPowershot = new PathFollow(new Waypoint[]{
                new Waypoint(STARTING, new FlywheelAction(true)),
                new Waypoint(POWERSHOT, new BluePowershotsAction())
        }, robot, "Start to powershot");

        ArrayList<Action> firstDropOffActions = new ArrayList<>();
        firstDropOffActions.add(new WobbleClawAction(false));
        firstDropOffActions.add(new FlywheelAction(true));
        PathFollow powershotToWobbleDropoff = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT, new WobbleArmAction(WobbleModule.WobbleArmPosition.AUTO_DROP)),
                new Waypoint(firstWobbleDropOff, firstDropOffActions)
        }, robot, "Powershot to first wobble dropoff");

        ArrayList<Action> secondWobbleActions = new ArrayList<>();
        secondWobbleActions.add(new IntakeAction(false));
        secondWobbleActions.add(new WobbleClawAction(true));
        secondWobbleActions.add(new SlowModeAction(true));
        PathFollow wobbleDropoffToSecondWobble = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff, new IntakeAction(true)),
                new Waypoint(STACK, new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED)),
                new Waypoint(SECOND_WOBBLE.x, (STACK.y - SECOND_WOBBLE.y) / 2),
                new Waypoint(SECOND_WOBBLE, secondWobbleActions)
        }, robot, "First wobble drop off to second wobble");

        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE, new SlowModeAction(false)),
                new Waypoint(secondWobbleDropoff.x, SHOOT_RING_Y, new ShootAction(BLUE_HIGH)),
                new Waypoint(secondWobbleDropoff, new WobbleClawAction(false))
        }, robot, "Second wobble to second wobble dropoff");

        PathFollow secondWobbleDropOffToPark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff, new WobbleArmAction(WobbleModule.WobbleArmPosition.RAISED)),
                new Waypoint(secondWobbleDropoff.x, PARK_Y)
        }, robot, "Second wobble drop off to park");

        startToPowershot.pathFollow(0, 1, 1, false, 0);
        sleep(1000);

        powershotToWobbleDropoff.pathFollow(0, 1, 1, true, Math.toRadians(-90));
        sleep(1000);

        wobbleDropoffToSecondWobble.pathFollow(0, 1, 1, true, Math.toRadians(180));
        sleep(1000);

        secondWobbleToSecondWobbleDropOff.pathFollow(0, 1, 1, true, 0);
        sleep(1000);

        secondWobbleDropOffToPark.pathFollow(0, 1, 1, true, 0);
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
