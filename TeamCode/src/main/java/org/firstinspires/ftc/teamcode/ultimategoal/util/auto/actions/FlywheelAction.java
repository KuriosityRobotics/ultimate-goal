package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class FlywheelAction extends Action {
    boolean isFlywheelOn;

    public FlywheelAction(boolean isFlywheelOn) {
        this.isFlywheelOn = isFlywheelOn;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.shooter.isFlyWheelOn = isFlywheelOn;

        return true;
    }

    @Override
    public String getName() {
        return "Flywheel Action";
    }
}
