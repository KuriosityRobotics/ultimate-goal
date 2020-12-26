package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.RobotAction;

public class SlowModeAction extends RobotAction {
    boolean isSlowMode;

    public SlowModeAction(boolean isSlowMode) {
        this.isSlowMode = isSlowMode;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.drivetrain.isSlowMode = isSlowMode;

        return true;
    }

    @Override
    public String getName() {
        return "SlowMode Action";
    }
}
