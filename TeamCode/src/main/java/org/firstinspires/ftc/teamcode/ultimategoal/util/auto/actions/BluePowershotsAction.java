package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;

public class BluePowershotsAction extends Action {
    Robot robot;

    int powershotNum = 0;

    @Override
    public boolean executeAction(Robot robot) {
        if (powershotNum < 3) {
            robot.drivetrain.setMovements(0, 0, 0); // stop

            robot.shooter.isAimBotActive = true;

            switch (powershotNum) {
                case 0:
                    robot.shooter.target = Target.Blue.BLUE_POWERSHOT1;
                    break;
                case 1:
                    robot.shooter.target = Target.Blue.BLUE_POWERSHOT2;
                    break;
                case 2:
                    robot.shooter.target = Target.Blue.BLUE_POWERSHOT3;
                    break;
            }

            if (robot.shooter.isFinishedIndexing()) {
                powershotNum++;
            }

            shootAndWait();

            return false;
        } else {
            robot.shooter.isAimBotActive = false;
            return true;
        }
    }

    private void shootAndWait() {
        robot.shooter.queueIndex();

        while ((!robot.shooter.isFinishedIndexing())) {

        }

//        robot.opModeSleep(5000);
    }

    @Override
    public String getName() {
        return "Powershots Action";
    }
}
