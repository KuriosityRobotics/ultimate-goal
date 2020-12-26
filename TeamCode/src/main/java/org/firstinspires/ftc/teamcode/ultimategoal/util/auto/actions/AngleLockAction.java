package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollow;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.PathFollowAction;

public class AngleLockAction extends PathFollowAction {
    int startIndex;
    int endIndex;
    double angleLockHeading;

    public AngleLockAction(double angleLockHeading, int startIndex, int endIndex) {
        this.angleLockHeading = angleLockHeading;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
    }

    @Override
    public boolean executeAction(PathFollow pathFollow) {
        int currentIndex = pathFollow.currentPathIndex();

        if (currentIndex > endIndex) {
            pathFollow.currentlyAngleLocking = false;

            return true;
        } else if (currentIndex > startIndex) {
            pathFollow.currentAngleLockHeading = angleLockHeading;
            pathFollow.currentlyAngleLocking = true;
        }

        return false;
    }

    @Override
    public String getName() {
        return "AngleLockAction";
    }
}
