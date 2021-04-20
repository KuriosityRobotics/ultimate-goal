package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class HopperDeliverAction extends Action {
    @Override
    public boolean executeAction(Robot robot) {
        robot.shooter.deliverRings();
        return true;
    }

    @Override
    public String getName() {
        return "HopperDeliverAction";
    }
}
