package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class BluePowershotsAction extends Action implements TelemetryProvider {
    Robot robot;

    int powershotNum = -1;

    public BluePowershotsAction() {
        this.stopMovementForExecution = true;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (beginExecutionTime == 0) {
            this.robot = robot;
            robot.telemetryDump.registerProvider(this);
            beginExecutionTime = robot.getCurrentTimeMilli();
        }

        if (robot.shooter.isFinishedIndexing()) {
            powershotNum++;

            if (powershotNum > 2) {
                robot.shooter.isAimBotActive = false;
                return true;
            }
        } else {
            return false;
        }

        switch (powershotNum) {
            case -1:
                break;
            case 0:
                robot.shooter.target = Target.Blue.BLUE_POWERSHOT1;
                break;
            case 1:
                robot.shooter.target = Target.Blue.BLUE_POWERSHOT2;
                break;
            case 2:
                robot.shooter.target = Target.Blue.BLUE_POWERSHOT3;
                break;
            default:
                return false;
        }

        robot.shooter.isAimBotActive = true;

        robot.shooter.queueIndex();

        return false;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("is shooter done: " + robot.shooter.isFinishedIndexing());
        data.add("powershot num: " + powershotNum);
        return data;
    }

    @Override
    public String getName() {
        return "Powershots Action";
    }
}
