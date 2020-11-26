package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class BluePowershotsAction extends Action {
    Robot robot;

    @Override
    public boolean executeAction(Robot robot) {
        this.robot = robot;

        robot.drivetrain.setMovements(0, 0, 0); // stop

        robot.shooter.isAimBotActive = true;

        robot.shooter.target = Target.Blue.BLUE_POWERSHOT1;
        shootAndWait();

        robot.shooter.nextTarget();
        shootAndWait();

        robot.shooter.nextTarget();
        shootAndWait();

        robot.opModeSleep(1000);

        robot.shooter.isAimBotActive = false;

        return true;
    }

    private void shootAndWait() {
        robot.shooter.queueIndex();

        while (!robot.shooter.isFinishedIndexing() && robot.isOpModeActive()) {}
    }

    @Override
    public String getName() {
        return "Powershots Action";
    }
}
