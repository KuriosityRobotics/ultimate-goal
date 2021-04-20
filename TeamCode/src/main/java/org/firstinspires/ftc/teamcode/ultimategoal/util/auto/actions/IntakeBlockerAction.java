package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class IntakeBlockerAction extends Action {
    IntakeModule.IntakeBlockerPosition position;

    public void IntakeBlockerAction(IntakeModule.IntakeBlockerPosition position) {
        this.position = position;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.intakeModule.blockerPosition = position;

        return true;
    }

    @Override
    public String getName() {
        return "IntakeBlockerAction";
    }
}
