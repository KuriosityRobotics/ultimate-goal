package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class DelayAction extends Action {
    long delayTime;
    Action action;

    public DelayAction(long delayTime, Action action) {
        this.delayTime = delayTime;
        this.action = action;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (beginExecutionTime == 0) {
            beginExecutionTime = robot.getCurrentTimeMilli();
        }

        if (robot.getCurrentTimeMilli() > beginExecutionTime + delayTime) {
            return action.executeAction(robot);
        } else {
            return false;
        }
    }

    @Override
    public String getName() {
        return "DelayAction";
    }
}
