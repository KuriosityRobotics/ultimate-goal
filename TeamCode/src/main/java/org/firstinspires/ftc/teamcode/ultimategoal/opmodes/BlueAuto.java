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
import org.opencv.core.Mat;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.Target.Blue.BLUE_HIGH;

@Autonomous
public class BlueAuto extends LinearOpMode implements TelemetryProvider {
    Robot robot;
    Vision vision;

    final Point STARTING = new Point(48 - 9, 0);

    final Point POWERSHOT = new Point(STARTING.x+5, 23.5 * 2.5);

    Vision.TargetGoal measuredZone;

    final Point TARGET_A_DROPOFF_FIRST = new Point(23 + 8, 80 - (16.5 / 2));
    final Point TARGET_B_DROPOFF_FIRST = new Point(23.5 * 2 - 2, 88 - (16.5 / 2));
    final Point TARGET_C_DROPOFF_FIRST = new Point(23 + 4, 108 - (16.5 / 2));

    Point firstWobbleDropOff;

    final Point STACK = new Point(34 - 9, 47 - (16.5 / 2));

    final Point SECOND_WOBBLE = new Point(36 - 13.5, 30.5-5);

    final double SHOOT_RING_Y = 60;

    final Point TARGET_A_DROPOFF_SECOND = new Point(18 - 9, 88 - 9);
    final Point TARGET_B_DROPOFF_SECOND = new Point(23 + 12 - 9, 94.25 - 18);
    final Point TARGET_C_DROPOFF_SECOND = new Point(18 - 13, 117 - 20);

    Point secondWobbleDropoff;

    final double PARK_Y = 82 - (16.5 / 2);

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
                new Waypoint(STACK.x+15,STACK.y),
                new Waypoint(firstWobbleDropOff, firstDropOffActions)
        }, robot, "startinng to first wobble dropoff");


        PathFollow firstWobbleToPowerShot = new PathFollow(new Waypoint[]{
                new Waypoint(firstWobbleDropOff, new FlywheelAction(true)),
                new Waypoint(POWERSHOT, new BluePowershotsAction())
        }, robot, "first wobble dropoff to powershot");

        ArrayList<Action> secondWobbleActions = new ArrayList<>();
        secondWobbleActions.add(new IntakeAction(true));
        secondWobbleActions.add(new WobbleClawAction(true));
        secondWobbleActions.add(new FlywheelAction(true));

        ArrayList<Action> secondWobbleStartActions = new ArrayList<>();
        secondWobbleStartActions.add(new IntakeAction(true));
        secondWobbleStartActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));

        PathFollow wobbleDropoffToSecondWobble = new PathFollow(new Waypoint[]{
                new Waypoint(POWERSHOT, secondWobbleStartActions),
                new Waypoint(STACK,new FlywheelAction(true)),
                new Waypoint(SECOND_WOBBLE, secondWobbleActions)
        }, robot, "Powershot to second wobble");

        ArrayList<Action> secondDropOffActions = new ArrayList<>();
        secondDropOffActions.add(new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED));
        secondDropOffActions.add(new IntakeAction(false));
        secondDropOffActions.add(new WobbleClawAction(false));
        PathFollow secondWobbleToSecondWobbleDropOff = new PathFollow(new Waypoint[]{
                new Waypoint(SECOND_WOBBLE,new WobbleArmAction(WobbleModule.WobbleArmPosition.WALL_DROP)),
                new Waypoint(secondWobbleDropoff, secondDropOffActions)
        }, robot, "Second wobble to second wobble dropoff");

        PathFollow secondWobbleDropOffToPark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff, new WobbleArmAction(WobbleModule.WobbleArmPosition.RAISED)),
                new Waypoint(secondWobbleDropoff.x, SHOOT_RING_Y),
        }, robot, "Second wobble drop off to park");

        PathFollow shotTopark = new PathFollow(new Waypoint[]{
                new Waypoint(secondWobbleDropoff),
                new Waypoint(secondWobbleDropoff.x, PARK_Y)
        }, robot, "Second wobble drop off to park");

        startToFirstWobbleDropOff.pathFollow(0, 1, 1, true, Math.toRadians(-45));
        sleep(500);

        firstWobbleToPowerShot.pathFollow(Math.toRadians(180), 1, 1, true,0);
        sleep(500);

        wobbleDropoffToSecondWobble.pathFollow(0, 0.8, 1, true, Math.toRadians(215));
        sleep(500);

        secondWobbleToSecondWobbleDropOff.pathFollow(0, 1, 1, true, 0);
        sleep(500);

        secondWobbleDropOffToPark.pathFollow(Math.toRadians(180), 1, 1, true, 0);

        shotTopark.pathFollow(0,1,1,true,0);
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
