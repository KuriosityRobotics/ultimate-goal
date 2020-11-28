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

    final Point POWERSHOT = new Point(STARTING.x + 5, 23.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23, 80 - 15);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 2, 88 - (16.5 / 2) + 6);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 + 4, 108 - (16.5 / 2));

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 9, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(36 - 13.5, 30.5 - 5);

    final double SHOOT_RING_Y = 60;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 9, 47 + 7);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 18);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18 - 13, 117 - 20);

    Point secondWobbleDropoff;

    final Point PARK = new Point(23 + 12 - 9, 82 - (16.5 / 2));
    final Point BEFOREPARK = new Point(23.5 * 2, 23.5 * 3);

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

//        PathFollow startTofirstDroppOffActions = new PathFollow(new Waypoint[]{
//                new Waypoint(STARTING, new FlywheelAction(true)),
//                new Waypoint(POWERSHOT, new BluePowershotsAction())
//        }, robot, "Start to powershot");

        ArrayList<Action> firstDropOffActions = new ArrayList<>();
        firstDropOffActions.add(new FlywheelAction(true));
        firstDropOffActions.add(new WobbleClawAction(false));
        firstDropOffActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.AUTO_DROP));
        ArrayList<Action> startActions = new ArrayList<>();
        startActions.add(new FlywheelAction(true));
        startActions.add(new WobbleClawAction(true));
        startActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.WALL_DROP));
        PathFollow startToFirstWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(STARTING, startActions),
                new Waypoint(STACK.x + 15, STACK.y),
                new Waypoint(firstWobbleDropOff, firstDropOffActions)
        }, robot, "startinng to first wobble dropoff");

        ArrayList<Action> leavingFirstWobbleActions = new ArrayList<>();
        leavingFirstWobbleActions.add(new FlywheelAction(true));
        leavingFirstWobbleActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.RAISED));
        PathFollow firstWobbleDropOffToPowerShot = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff, leavingFirstWobbleActions),
                new Waypoint(POWERSHOT, new BluePowershotsAction())
        }, robot, "first wobble dropoff to powershot");

        ArrayList<Action> secondWobbleActions = new ArrayList<>();
        secondWobbleActions.add(new IntakeAction(false));
        secondWobbleActions.add(new WobbleClawAction(true));

        ArrayList<Action> secondWobbleStartActions = new ArrayList<>();
        secondWobbleStartActions.add(new IntakeAction(true));
        secondWobbleStartActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));

        PathFollow powerShotToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow powerShotToStack = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow backFromStack = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow stackToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
        if (measuredZone == Vision.TargetGoal.C) {
            powerShotToStack = new PathFollow(new Waypoint[]{
                    new Waypoint(POWERSHOT, secondWobbleStartActions),
                    new Waypoint(POWERSHOT.x + 64, POWERSHOT.y + 12),
                    new Waypoint(STACK),
                    new Waypoint(STACK.x - 2, STACK.y)
            }, robot, "Powershot to stack");

            backFromStack = new PathFollow(new Waypoint[]{
                    new Waypoint(STACK.x - 2, STACK.y),
                    new Waypoint(STACK.x + 12, STACK.y)
            }, robot, "Back from stack");

            stackToSecondWobble = new PathFollow(new Waypoint[]{
                    new Waypoint(SECOND_WOBBLE.x, SECOND_WOBBLE.y + 8),
                    new Waypoint(SECOND_WOBBLE, secondWobbleActions)
            }, robot, "Stack to second wobble");
        } else {
            powerShotToSecondWobble = new PathFollow(new Waypoint[]{
                    new Waypoint(POWERSHOT, secondWobbleStartActions),
                    new Waypoint(STACK),
                    new Waypoint(SECOND_WOBBLE, secondWobbleActions)
            }, robot, "Powershot to second wobble");
        }

        ArrayList<Action> secondDropOffActions = new ArrayList<>();
        secondDropOffActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));
        secondDropOffActions.add(new WobbleClawAction(false));
        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE, new WobbleArmAction(WobbleModule.WobbleArmPosition.WALL_DROP)),
                new Waypoint(secondWobbleDropoff, secondDropOffActions)
        }, robot, "Second wobble to second wobble dropoff");

        ArrayList<Action> fromSecondWobbleActions = new ArrayList<>();
        fromSecondWobbleActions.add(new FlywheelAction(true));
        fromSecondWobbleActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.RAISED));
        PathFollow secondWobbleDropOffToShoot = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff, fromSecondWobbleActions),
                new Waypoint(secondWobbleDropoff.x, SHOOT_RING_Y, new ShootAction(BLUE_HIGH)),
        }, robot, "Second wobble drop off to park");

        PathFollow shootToPark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff),
                new Waypoint(BEFOREPARK),
                new Waypoint(PARK)
        }, robot, "Second wobble drop off to park");

        startToFirstWobbleDropOff.pathFollow(0, 1, 1, true, Math.toRadians(-45));
        sleep(500);

        firstWobbleDropOffToPowerShot.pathFollow(Math.toRadians(180), 1, 1, true, 0);
        sleep(500);

        if (measuredZone == Vision.TargetGoal.C) {
            robot.drivetrain.brakeHeading = Math.toRadians(90);
            sleep(500);

            powerShotToStack.pathFollow(0, 1, 1, true, Math.toRadians(-90));

            backFromStack.pathFollow(Math.toRadians(180), 1, 1, false, 0);

            robot.drivetrain.brakeHeading = Math.toRadians(-110);
            sleep(750);

            stackToSecondWobble.pathFollow(0, 1, 1, true, Math.toRadians(215));
        } else {
            powerShotToSecondWobble.pathFollow(0, 0.8, 1, true, Math.toRadians(215));
        }
        sleep(500);

        secondWobbleToSecondWobbleDropOff.pathFollow(0, 1, 1, true, 0);
        sleep(500);

        secondWobbleDropOffToShoot.pathFollow(Math.toRadians(180), 1, 1, false, 0);

        shootToPark.pathFollow(0, 1, 1, true, 0);
        sleep(2000);
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
