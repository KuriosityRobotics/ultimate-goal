package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.RobotAction;

public class WobbleClawAction extends RobotAction {
    boolean isClamped;

    public WobbleClawAction(boolean isClamped) {
        this.isClamped = isClamped;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.wobbleModule.isClawClamped = isClamped;

        return true;
    }

    @Override
    public String getName() {
        return "Wobble Claw Action";
    }
}
