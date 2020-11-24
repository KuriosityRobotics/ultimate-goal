package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class SlowModeAction extends Action {
    @Override
    public boolean executeAction(Robot robot) {
        robot.drivetrain.isSlowMode = true;

        return true;
    }

    @Override
    public String getName() {
        return "SlowMode Action";
    }
}
