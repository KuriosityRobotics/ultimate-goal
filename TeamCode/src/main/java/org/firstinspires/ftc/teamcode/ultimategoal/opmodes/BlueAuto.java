package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.BluePowershotsAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.FlywheelAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.IntakeBlockerAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.RunIntakeAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootStackAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.Vision;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_POWERSHOT1;

@Autonomous
public class BlueAuto extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    Vision vision;

    static final Pose2d STARTING = new Pose2d(48 - 9, 0, new Rotation2d(0));

    public static final Point POWERSHOT = new Point(STARTING.getTranslation().getX() + 5, 20.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23, 80 - 19);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 2, 88 - (16.5 / 2) + 6);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 - 4, 112 - (16.5 / 2) + 2);

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 9, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(36 - 18, 30.5 - 4);

    final double SHOOT_RING_Y = 60;
    final double SHOOT_RING_X = 28;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 12, 47 + 4);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 18);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18 - 15, 117 - 20);

    Point secondWobbleDropoff;

    public static final Pose2d PARK = new Pose2d(23 + 12 - 9, 82 - (16.5 / 2), new Rotation2d(0));
    final Point BEFOREPARK = new Point(23.5 * 2, 23.5 * 3);

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, STARTING);

        vision = new Vision(this);

        robot.telemetryDump.registerProvider(this);

        waitForStart();

//        robot.shooter.target = BLUE_POWERSHOT1;
//        robot.shooter.lockTarget = false;
//        robot.shooter.setTurretTargetHeading(0);

        measuredZone = vision.runDetection();
        measuredZone = Vision.TargetGoal.C;
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
        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.INIT;
//        PathFollow startTofirstDroppOffActions = new PathFollow(new Waypoint[]{
//                new Waypoint(new Point(STARTING.getTranslation()), new FlywheelAction(true)),
//                new Waypoint(POWERSHOT, new BluePowershotsAction())
//        }, robot, "Start to powershot");

//        ArrayList<Action> powershotActions = new ArrayList<>();
//        powershotActions.add(new ShootAction(BLUE_HIGH));
//        powershotActions.add(new FlywheelAction(true));

        ArrayList<Action> startActions = new ArrayList<>();
        startActions.add(new FlywheelAction(true));

        PathFollow startToPowershot = new PathFollow(new Waypoint[]{
                new Waypoint(new Point(STARTING.getTranslation().getX(), STARTING.getTranslation().getY()), startActions),
                new Waypoint(STACK.x + 15, STACK.y),
                new Waypoint(POWERSHOT)
        }, robot, "startinng to first wobble dropoff");

        ArrayList<Action> powershotToFirstWobbleStartActions = new ArrayList<>();
        powershotToFirstWobbleStartActions.add(new FlywheelAction(true));

        PathFollow powershotToFirstWobble = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT, powershotToFirstWobbleStartActions),
                new Waypoint(firstWobbleDropOff, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.STREAMLINE))
        }, robot, "first wobble dropoff to powershot");

        PathFollow backFromFirstWobble = new PathFollow(new Waypoint[] {
                new Waypoint(firstWobbleDropOff),
                new Waypoint(firstWobbleDropOff.x + 16, firstWobbleDropOff.y)
        }, robot, "back away from first wobble");

        PathFollow towardsStack = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff.x + 16, firstWobbleDropOff.y),
                new Waypoint(STACK.x, STACK.y + 16, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.BLOCKING)),
                new Waypoint(STACK.x, STACK.y + 8)
        }, robot, "towards the stack");

        ArrayList<Action> secondWobbleStartActions = new ArrayList<>();
        secondWobbleStartActions.add(new RunIntakeAction(false));
        secondWobbleStartActions.add(new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.STREAMLINE));

        ArrayList<Action> secondWobbleEndActions = new ArrayList<>();
        secondWobbleEndActions.add(new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.WOBBLE));
        secondWobbleEndActions.add(new RunIntakeAction(false));
        secondWobbleEndActions.add(new FlywheelAction(false));

//        PathFollow powerShotToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
//        PathFollow firstwobbleToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
//        PathFollow backFromStack = new PathFollow(new Waypoint[]{}, robot, "filler");
//        PathFollow stackToSecondWobble = new PathFollow(new Waypoint[]{}, robot, "filler");
//        PathFollow stackToSecondWobble2 = new PathFollow(new Waypoint[]{}, robot, "filler");
//
//        firstwobbleToSecondWobble = new PathFollow(new Waypoint[]{
//                new Waypoint(firstWobbleDropOff, secondWobbleStartActions),
//                new Waypoint(SECOND_WOBBLE.x, SECOND_WOBBLE.y + 55, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.OPEN)),
//                new Waypoint(SECOND_WOBBLE.x, SECOND_WOBBLE.y + 8, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.WOBBLE)),
//                new Waypoint(SECOND_WOBBLE, secondWobbleEndActions),
//        }, robot, "Powershot to stack");
//
//        if (measuredZone == Vision.TargetGoal.C) {
//
//
//            backFromStack = new PathFollow(new Waypoint[]{
//                    new Waypoint(POWERSHOT.x + 6, STACK.y),
//                    new Waypoint(POWERSHOT.x + 4, STACK.y + 2),
//                    new Waypoint(STACK.x - 7, STACK.y + 3)
//            }, robot, "Back from stack");
//
//            stackToSecondWobble = new PathFollow(new Waypoint[]{
//                    new Waypoint(STACK.x - 7, STACK.y + 3),
//                    new Waypoint(STACK.x + 2, STACK.y),
//            }, robot, "Stack to second wobble");
//            stackToSecondWobble2 = new PathFollow(new Waypoint[]{
//                    new Waypoint(STACK.x + 2, STACK.y),
//                    new Waypoint(SECOND_WOBBLE, secondWobbleActions)
//            }, robot, "Stack to second wobble");
//        } else {
//            powerShotToSecondWobble = new PathFollow(new Waypoint[]{
//                    new Waypoint(POWERSHOT, secondWobbleStartActions),
//                    new Waypoint(STACK.x + 4, STACK.y),
//                    new Waypoint(STACK),
//                    new Waypoint(SECOND_WOBBLE, secondWobbleActions)
//            }, robot, "Powershot to second wobble");
//        }

        PathFollow stackToSecondWobble = new PathFollow(new Waypoint[]{
                new Waypoint(STACK.x - 6, STACK.y - 12),
                new Waypoint(SECOND_WOBBLE, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.WOBBLE))
        }, robot, "stack to second wobble");

        ArrayList<Action> secondDropOffActions = new ArrayList<>();
        secondDropOffActions.add(new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.OPEN));

        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE),
                new Waypoint(secondWobbleDropoff, secondDropOffActions)
        }, robot, "Second wobble to second wobble dropoff");

        PathFollow secondDropOffToPark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff),
                new Waypoint(new Point(PARK.getTranslation()), new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.BLOCKING))
        }, robot, "second drop off to parking");
//
//        ArrayList<Action> fromSecondWobbleActions = new ArrayList<>();
//        fromSecondWobbleActions.add(new FlywheelAction(true));
//
//        ArrayList<Action> shootActions = new ArrayList<>();
//        shootActions.add(new ShootStackAction(2, new Point(SHOOT_RING_X,SHOOT_RING_Y), BLUE_HIGH));
//
//
//        PathFollow secondWobbleDropOffToShoot = new PathFollow(new Waypoint[]{
//                new Waypoint(secondWobbleDropoff, fromSecondWobbleActions),
//                new Waypoint(SHOOT_RING_X, SHOOT_RING_Y, shootActions),
//        }, robot, "Second wobble drop off to park");
//
//        PathFollow shootToPark = new PathFollow(new Waypoint[]{
//                new Waypoint(SHOOT_RING_X, SHOOT_RING_Y),
//                new Waypoint(new Point(PARK.getTranslation().getX(), PARK.getTranslation().getY()))
//        }, robot, "Second wobble drop off to park");

        sleep(750);
        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.WOBBLE;

        startToPowershot.followPath(0, 1, 1, true, Math.toRadians(0), true);

        BluePowershotsAction powershotsAction = new BluePowershotsAction();
        while (!powershotsAction.executeAction(robot) && opModeIsActive()) {
            Log.v("blueauto", "poweraction");
        }

        powershotToFirstWobble.followPath(0, 1, 1, true, Math.toRadians(-45));
        sleep(500);

        long startTime = SystemClock.elapsedRealtime();
        long currentTime = startTime;
        while (currentTime < startTime + 250 && opModeIsActive()) {
            robot.drivetrain.setMovements(0.4, -0.6, -0.4);
            currentTime = SystemClock.elapsedRealtime();
        }
//        backFromFirstWobble.followPath(Math.toRadians(180), 1, 1, false, Math.toRadians(-45));

        towardsStack.followPath(0, 1, 1, true, Math.toRadians(180), true);

        ShootStackAction shootStackAction = new ShootStackAction(4, new Point(STACK.x - 6, STACK.y - 12), BLUE_HIGH);
        while (!shootStackAction.executeAction(robot) && opModeIsActive()) {
            // yeet
            Log.v("shootstack", "action");
        }

//        firstwobbleToSecondWobble.followPath(0, 1, 1, true, Math.toRadians(-150));

        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.INIT;
        sleep(500);

        stackToSecondWobble.followPath(0, 1, 1, true, Math.toRadians(180));
        sleep(500);

        robot.drivetrain.setBrakeHeading(Math.toRadians(90));
        sleep(100);
        robot.drivetrain.setBrakeHeading(0);
        while (Math.abs(robot.drivetrain.getCurrentHeading() - robot.drivetrain.getBrakeHeading()) > Math.toRadians(4) && opModeIsActive()) {
            // wait
        }

        robot.shooter.lockTarget = false;
        robot.shooter.setTurretTargetHeading(0);

        secondWobbleToSecondWobbleDropOff.followPath(0, 1, 1, true, 0);
        sleep(500);

        secondDropOffToPark.followPath(0, 1, 1, true, 0);
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
//            powerShotToSecondWobble.followPath(0, 0.55, 1, true, Math.toRadians(215));
//        }
//        sleep(500);
//
//        secondWobbleToSecondWobbleDropOff.followPath(0, 1, 1, true, 0);
//        sleep(500);
//
//        secondWobbleDropOffToShoot.followPath(Math.toRadians(180), 1, 1, false, 0);
//
//        shootToPark.followPath(0, 1, 1, true, 0);
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
