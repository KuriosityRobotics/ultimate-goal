package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.AutoShootAction;
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

    static final Point STARTING = new Point(48 - 9, 0);

    public static final Point POWERSHOT = new Point(STARTING.x + 5, 23.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23, 80 - 19);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 2, 88 - (16.5 / 2) + 6);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 + 2, 112 - (16.5 / 2));

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 9, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(36 - 16, 30.5 - 4);

    final double SHOOT_RING_Y = 60;
    final double SHOOT_RING_X = 28;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 12, 47 + 4);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 18);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18 - 15, 117 - 20);

    Point secondWobbleDropoff;

    public static final Point PARK = new Point(23 + 12 - 9, 82 - (16.5 / 2));
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
        firstDropOffActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.AUTO_DROP));
        firstDropOffActions.add(new FlywheelAction(true));
        firstDropOffActions.add(new WobbleClawAction(false));
        ArrayList<Action> startActions = new ArrayList<>();
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
        secondWobbleActions.add(new FlywheelAction(true));


        ArrayList<Action> secondWobbleStartActions = new ArrayList<>();
        secondWobbleStartActions.add(new IntakeAction(false));
        secondWobbleStartActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));

        PathFollow powerShotToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow powerShotToStack = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow backFromStack = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow stackToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
        PathFollow stackToSecondWobble2 = new PathFollow(new Waypoint[]{}, robot, "filler");

        if (measuredZone == Vision.TargetGoal.C) {
            powerShotToStack = new PathFollow(new Waypoint[]{
                    new Waypoint(POWERSHOT, secondWobbleStartActions),
                    new Waypoint(POWERSHOT.x + 6, STACK.y),
            }, robot, "Powershot to stack");

            backFromStack = new PathFollow(new Waypoint[]{
                    new Waypoint(POWERSHOT.x + 6, STACK.y),
                    new Waypoint(POWERSHOT.x + 4, STACK.y + 2),
                    new Waypoint(STACK.x - 7, STACK.y + 3)
            }, robot, "Back from stack");

            stackToSecondWobble = new PathFollow(new Waypoint[]{
                    new Waypoint(STACK.x - 7, STACK.y + 3),
                    new Waypoint(STACK.x + 2, STACK.y),
            }, robot, "Stack to second wobble");
            stackToSecondWobble2 = new PathFollow(new Waypoint[]{
                    new Waypoint(STACK.x + 2, STACK.y),
                    new Waypoint(SECOND_WOBBLE, secondWobbleActions)
            }, robot, "Stack to second wobble");
        } else {
            powerShotToSecondWobble = new PathFollow(new Waypoint[]{
                    new Waypoint(POWERSHOT, secondWobbleStartActions),
                    new Waypoint(STACK.x + 25, STACK.y),
                    new Waypoint(SECOND_WOBBLE.x+6, SECOND_WOBBLE.y-8, secondWobbleActions)
            }, robot, "Powershot to second wobble");
        }

        ArrayList<Action> secondDropOffActions = new ArrayList<>();
        secondDropOffActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));
        secondDropOffActions.add(new IntakeAction(true));
        secondDropOffActions.add(new WobbleClawAction(false));
        secondDropOffActions.add(new FlywheelAction(false));


        ArrayList<Action> secondDropOffStartActions = new ArrayList<>();
        secondDropOffStartActions.add(new IntakeAction(true));
        secondDropOffStartActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.AUTO_DROP));

        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE.x+6,SECOND_WOBBLE.y-8, secondDropOffStartActions),
                new Waypoint(STACK.x, STACK.y - 20)
        }, robot, "Second wobble to second wobble dropoff");

        ArrayList<Action> fromSecondWobbleActions = new ArrayList<>();
        fromSecondWobbleActions.add(new IntakeAction(true));

        ArrayList<Action> shootActions = new ArrayList<>();
        shootActions.add(new IntakeAction(true));
        shootActions.add(new AutoShootAction(BLUE_HIGH));
        shootActions.add(new FlywheelAction(true));

        ArrayList<Action> shootAction2 = new ArrayList<>();
        shootAction2.add(new IntakeAction(false));
        shootAction2.add(new AutoShootAction(BLUE_HIGH));

        PathFollow secondWobbleDropOffToShoot = new PathFollow(new Waypoint[]{
                new Waypoint(STACK.x, STACK.y - 20, fromSecondWobbleActions),
                new Waypoint(STACK.x, STACK.y-6.5),
        }, robot, "Second wobble drop off to park");

        ArrayList<Action> afterFirstShotActions = new ArrayList<>();
        afterFirstShotActions.add(new IntakeAction(true));
        afterFirstShotActions.add(new FlywheelAction(true));

        PathFollow shootToPark = new PathFollow(new Waypoint[]{
                new Waypoint(STACK.x, STACK.y-6.5,shootActions),
                new Waypoint(STACK.x,STACK.y+8, afterFirstShotActions)
        }, robot, "Second wobble drop off to park");

        PathFollow shootToPark2 = new PathFollow(new Waypoint[]{
                new Waypoint(STACK.x, STACK.y+8,shootAction2),
                new Waypoint(secondWobbleDropoff,secondDropOffActions)
        }, robot, "Second wobble drop off to park");

        startToFirstWobbleDropOff.followPath(0, 1, 1, true, Math.toRadians(-45));

        firstWobbleDropOffToPowerShot.followPath(Math.toRadians(180), 1, 1, true, 0);

//        if (measuredZone == Vision.TargetGoal.C) {
//            robot.drivetrain.brakeHeading = Math.toRadians(90);
//            sleep(500);
//
//            powerShotToStack.followPath(Math.toRadians(180), 1, 1, true, Math.toRadians(-90));
//
//            backFromStack.followPath(0, 1, 1, true, Math.toRadians(-90));
//
//            stackToSecondWobble.followPath(0, 1, 1, true, Math.toRadians(215));
//
//            stackToSecondWobble2.followPath(0, 0.55, 1, true, Math.toRadians(215));
//        } else {
        robot.shooter.setFlyWheelTargetSpeed(Shooter.HIGHGOAL_FLYWHEEL_SPEED);
            powerShotToSecondWobble.followPath(0, 0.7, 1, true, Math.toRadians(250));
//        }



        secondWobbleToSecondWobbleDropOff.followPath(0, 1, 1, true, 0);

        secondWobbleDropOffToShoot.followPath(Math.toRadians(0), 1, 1, true, 0);

        sleep(750);

        shootToPark.followPath(0, 1, 1, true, 0);

        sleep(750);

        shootToPark2.followPath(0,1,1,true,0);

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
