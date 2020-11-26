package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class WobbleClawAction extends Action {
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
