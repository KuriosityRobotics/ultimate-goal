package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class ShootAction extends Action {
    Target.ITarget target;

    public ShootAction(Target.ITarget target) {
        this.target = target;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.drivetrain.setMovements(0, 0, 0);

        robot.shooter.target = target;

        robot.shooter.isAimBotActive = true;

        robot.shooter.queueIndexThreeRings();

        while (!robot.shooter.isFinishedIndexing() && robot.isOpModeActive()) {}

        robot.opModeSleep(1000);

        robot.shooter.isAimBotActive = false;

        return true;
    }

    @Override
    public String getName() {
        return "Shoot Action";
    }
}
