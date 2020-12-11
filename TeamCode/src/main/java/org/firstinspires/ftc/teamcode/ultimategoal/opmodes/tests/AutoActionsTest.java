package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Waypoint;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ShootAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.SlowModeAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleArmAction;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.WobbleClawAction;

@Autonomous
public class AutoActionsTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        PathFollow pf1 = new PathFollow(new Waypoint[]{
                new Waypoint(0, 0),
                new Waypoint(0, 24, new SlowModeAction(true)),
                new Waypoint(0, 60, new WobbleArmAction(WobbleModule.WobbleArmPosition.LOWERED))
        }, robot, "test");

        waitForStart();

        robot.startModules();

        sleep(1000);

        pf1.pathFollow(0, 1, 1, true, 0);

        sleep(5000);
    }
}
