package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.RobotAction;

public class FlywheelAction extends RobotAction {
    boolean isFlywheelOn;

    public FlywheelAction(boolean isFlywheelOn) {
        this.isFlywheelOn = isFlywheelOn;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (isFlywheelOn) {
            robot.shooter.setFlyWheelSpeed(robot.FLY_WHEEL_SPEED);
        } else {
            robot.shooter.setFlyWheelSpeed(0);
        }
        return true;
    }

    @Override
    public String getName() {
        return "Flywheel Action";
    }
}
