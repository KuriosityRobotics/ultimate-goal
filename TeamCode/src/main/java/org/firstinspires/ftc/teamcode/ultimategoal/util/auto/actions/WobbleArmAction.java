package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.RobotAction;

public class WobbleArmAction extends RobotAction {
    WobbleModule.WobbleArmPosition wobbleArmPosition;

    public WobbleArmAction(WobbleModule.WobbleArmPosition wobbleArmPosition) {
        this.wobbleArmPosition = wobbleArmPosition;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.wobbleModule.wobbleArmPosition = this.wobbleArmPosition;

        return true;
    }

    @Override
    public String getName() {
        return "Wobble Arm Action";
    }
}
