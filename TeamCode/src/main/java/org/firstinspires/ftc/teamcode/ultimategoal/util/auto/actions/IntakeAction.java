package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class IntakeAction extends Action {
    boolean isIntakeOn;

    public IntakeAction(boolean isIntakeOn) {
        this.isIntakeOn = isIntakeOn;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (isIntakeOn) {
            robot.intakeModule.intakePower = 1;
        } else {
            robot.intakeModule.intakePower = 0;
        }

        return true;
    }

    @Override
    public String getName() {
        return "Intake Action";
    }
}
