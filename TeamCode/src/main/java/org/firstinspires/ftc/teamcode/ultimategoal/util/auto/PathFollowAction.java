package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

/**
 * An action that modifies a PathFollow object.
 */
public abstract class PathFollowAction extends Action {
    /**
     * Execute the action, returning whether or not the action has finished executing
     *
     * @param pathFollow The PathFollower object to adjust
     * @return Whether or not the action has completed executing
     */
    public abstract boolean executeAction(PathFollow pathFollow);
}
