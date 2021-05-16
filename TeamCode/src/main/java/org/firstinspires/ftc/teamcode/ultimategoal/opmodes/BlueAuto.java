package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import android.os.SystemClock;

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
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.FlywheelAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.IntakeBlockerAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootStackAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.Vision;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;

@Autonomous
public class BlueAuto extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    Vision vision;

    static final Pose2d STARTING = new Pose2d(48 - 9, 0, new Rotation2d(0));

    public static final Point POWERSHOT = new Point(STARTING.getTranslation().getX(), 20.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23 - 12, 80 - 9);
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 6, 88 - (16.5 / 2) + 16);
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 - 9, 112 - (16.5 / 2) + 11);

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 7, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(36 - 18 - 0, 30.5 - 3);

    final double SHOOT_RING_Y = 60;
    final double SHOOT_RING_X = 28;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 10, 47 + 9);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 16);
    final Point TARGET_C_DROPOFF_SECOND = new Point(14 - 8, 117 - 20 + 5);

    Point secondWobbleDropoff;

    public static final Pose2d PARK = new Pose2d(23 + 12 - 9, 82 - (16.5 / 2), new Rotation2d(0));
    final Point BEFOREPARK = new Point(23.5 * 2, 23.5 * 3);

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this, STARTING);

        vision = new Vision(this);

        robot.telemetryDump.registerProvider(this);

        waitForStart();

        robot.shooter.manualTurretPower = 0;
        robot.shooter.manualTurret = false;

        robot.shooter.manualAngleCorrection = 0.000;
        robot.shooter.manualAngleFlapCorrection = -0.002;

        measuredZone = vision.runDetection();
//        measuredZone = Vision.TargetGoal.C;
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

        ArrayList<Action> startActions = new ArrayList<>();
        startActions.add(new FlywheelAction(true));
        PathFollow startToPowershot = new PathFollow(new Waypoint[]{
                new Waypoint(new Point(STARTING.getTranslation()), startActions),
                new Waypoint(STACK.x + 16, STACK.y, new ShootAction(BLUE_HIGH, false)),
                new Waypoint(POWERSHOT)
        }, robot, "startinng to first wobble dropoff");

        ArrayList<Action> powershotToFirstWobbleStartActions = new ArrayList<>();
        powershotToFirstWobbleStartActions.add(new FlywheelAction(true));

        PathFollow powershotToFirstWobble = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT, powershotToFirstWobbleStartActions),
                new Waypoint(firstWobbleDropOff, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.STREAMLINE))
        }, robot, "first wobble dropoff to powershot");

        ArrayList<Action> towardsStackActions = new ArrayList<>();
        towardsStackActions.add(new DelayAction(500, new IntakeBlockerAction(IntakeModule.IntakeBlockerPosition.BLOCKING)));
        if (measuredZone != Vision.TargetGoal.A) {
            towardsStackActions.add(new DelayAction(900, new Action() {
                @Override
                public boolean executeAction(Robot robot) {
                    robot.shooter.manualTurret = false;
                    return true;
                }

                @Override
                public String getName() {
                    return "manual turret off";
                }
            }));
        }

        PathFollow towardsStack = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff.x + 16, firstWobbleDropOff.y, towardsStackActions),
                new Waypoint(STACK.x + 8, STACK.y + 20)
        }, robot, "towards the stack");

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
                new Waypoint(new Point(PARK.getTranslation()))
        }, robot, "second drop off to parking");

        sleep(400);
        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.WOBBLE;

        startToPowershot.followPath(0, 0.85, 0.9, false, Math.toRadians(0), true);

//        BluePowershotsAction powershotsAction = new BluePowershotsAction();
//        ShootAction powershotsAction = new ShootAction(BLUE_HIGH);
//        while (!powershotsAction.executeAction(robot) && opModeIsActive()) {
////            Log.v("blueauto", "poweraction");
//        }

        robot.shooter.manualTurret = true;

        if (measuredZone == Vision.TargetGoal.A) {
            while (Math.abs(angleWrap(robot.drivetrain.getCurrentHeading() - Math.toRadians(-45))) > Math.toRadians(10) && opModeIsActive()) {
                robot.drivetrain.setMovements(0, 0.2, -0.3);
            }

            powershotToFirstWobble.followPath(0, 0.45, 0.08, true, Math.toRadians(-45));
            sleep(1000);
        } else {
            powershotToFirstWobble.followPath(0, 1, 1, true, Math.toRadians(-45));
        }
//        sleep(500);

        long startTime = SystemClock.elapsedRealtime();
        long currentTime = startTime;
        while (currentTime < startTime + 400 && opModeIsActive()) {
            robot.drivetrain.setMovements(0.4, -0.6, -0.4);
            currentTime = SystemClock.elapsedRealtime();
        }

        robot.shooter.target = BLUE_HIGH;
        robot.shooter.manualAngleFlapCorrection = -0.00195;
        robot.shooter.manualAngleCorrection = +0.025;

        towardsStack.followPath(0, 0.85, 0.26, false, Math.toRadians(180), false);

        if (measuredZone == Vision.TargetGoal.A) {
            robot.drivetrain.setBrakeHeading(Math.toRadians(-170));
            while (Math.abs(angleWrap(robot.drivetrain.getCurrentHeading() - robot.drivetrain.getBrakeHeading())) > Math.toRadians(10) && opModeIsActive()) {
                // wait itj
            }
        }

        int ringsToExpect;
        switch (measuredZone) {
            case A:
                ringsToExpect = 0;
                break;
            case B:
                ringsToExpect = 1;
                break;
            default:
            case C:
                ringsToExpect = 4;
                break;
        }
        ShootStackAction shootStackAction = new ShootStackAction(ringsToExpect, new Point(STACK.x - 3, STACK.y - 4), BLUE_HIGH);
        while (!shootStackAction.executeAction(robot) && opModeIsActive()) {
            // yeet
        }
//        while ((!robot.shooter.isFinishedFiringQueue() || robot.shooter.getCurrentHopperPosition() != HopperModule.HopperPosition.LOWERED)
//                && opModeIsActive()) {
//            // yeeter
//        }

        robot.intakeModule.stopIntake = true;
        robot.shooter.queueIndex(3);
        robot.intakeModule.intakePower = 0;

        robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.OPEN;
        sleep(100);

        if (measuredZone == Vision.TargetGoal.A) {
            stackToSecondWobble.followPath(0, 0.3, 0.8, false, Math.toRadians(200), true);
            sleep(1000);

            long start = SystemClock.elapsedRealtime();
            long time = startTime;
            while (time < start + 400 && opModeIsActive()) {
                robot.drivetrain.setMovements(0, 0.1, 0);
                time = SystemClock.elapsedRealtime();
            }
        } else {
            stackToSecondWobble.followPath(0, 0.5, 1, false, Math.toRadians(200), true);
            sleep(300);
        }

        if (measuredZone == Vision.TargetGoal.A) {
            while (Math.abs(angleWrap(robot.drivetrain.getCurrentHeading() - 0)) > Math.toRadians(28) && opModeIsActive()) {
                robot.drivetrain.setMovements(0, 0.15, 0.4);
            }
        } else {
            robot.drivetrain.setBrakeHeading(0);
            while (Math.abs(angleWrap(robot.drivetrain.getCurrentHeading() - robot.drivetrain.getBrakeHeading())) > Math.toRadians(28) && opModeIsActive()) {
//            robot.drivetrain.setMovements(0, 0, -0.9);
                // wait
//            Log.v("blueauto", "waitin");
            }
        }

        robot.shooter.flywheelOn = false;
        robot.shooter.lockTarget = false;
        robot.shooter.manualTurret = true;

        if (measuredZone == Vision.TargetGoal.A) {
            secondWobbleToSecondWobbleDropOff.followPath(0, 1, 1, true, 0);
        } else {
            secondWobbleToSecondWobbleDropOff.followPath(0, 1, 1, false, 0);
        }
//        sleep(150);

        if (measuredZone == Vision.TargetGoal.A) {
            sleep(750);

            robot.drivetrain.setBrake(16, 36, Math.toRadians(15));
            while (robot.drivetrain.distanceToBrake() > 5 && opModeIsActive()) {
                // wait
            }

            robot.intakeModule.blockerPosition = IntakeModule.IntakeBlockerPosition.BLOCKING;

            robot.drivetrain.setBrake(28, 36, Math.toRadians(15));
            while (robot.drivetrain.distanceToBrake() > 4 && opModeIsActive()) {
                // wait
            }

            Point parkPoint = new Point(PARK.getTranslation().getX(), PARK.getTranslation().getY());
            while (robot.drivetrain.distanceToPoint(parkPoint) > 4 && opModeIsActive()) {
                robot.drivetrain.setMovementsTowardsPoint(parkPoint, 0.35, 0.2, Math.toRadians(180), true, 0);
            }
            robot.drivetrain.setMovements(0, 0, 0);
        } else if (measuredZone == Vision.TargetGoal.B) {
            robot.drivetrain.setBrake(new Point(PARK.getTranslation()), 0);
            Point parkPoint = new Point(PARK.getTranslation().getX(), PARK.getTranslation().getY() - 14);
            while (robot.drivetrain.distanceToPoint(parkPoint) > 4 && opModeIsActive()) {
                robot.drivetrain.setMovementsTowardsPoint(parkPoint, 0.3, 0.2, Math.toRadians(180), true, 0);
            }
            robot.drivetrain.setMovements(0, 0, 0);
        } else {
            secondDropOffToPark.followPath(Math.toRadians(180), 0.8, 1, true, 0);

            robot.drivetrain.setBrake(new Point(PARK.getTranslation()), 0);
        }

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
