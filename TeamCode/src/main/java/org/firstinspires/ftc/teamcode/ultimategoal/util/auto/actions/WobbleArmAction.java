package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class WobbleArmAction extends Action {
    boolean lowerWobble;

    public WobbleArmAction(boolean lowerWobble) {
        this.lowerWobble = lowerWobble;
    }

    @Override
    public boolean executeAction(Robot robot) {
        robot.wobbleModule.setWobbleArmPosition(WobbleModule.WobbleArmPosition.LOWERED);

        return true;
    }

    @Override
    public String getName() {
        return "Wobble Arm Action";
    }
}
