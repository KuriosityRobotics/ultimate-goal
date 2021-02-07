package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;


import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class AutoShootAction extends Action {
    Target.ITarget target;

    public AutoShootAction(Target.ITarget target) {
        this.target = target;
        this.stopMovementForExecution = true;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (beginExecutionTime == 0) {
            robot.shooter.target = target;

            robot.shooter.isAimBotActive = true;

            robot.shooter.queueIndexThreeRings();

            beginExecutionTime = robot.getCurrentTimeMilli();

            return false;
        } else {
            if (robot.shooter.isFinishedIndexing()) {
                robot.shooter.isAimBotActive = false;
                robot.shooter.isFlyWheelOn = true;
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    public String getName() {
        return "Shoot Action";
    }
}
