package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

/**
 * A class to keep track of an action, along with the state of its execution.
 */
public abstract class Action {
    protected long beginExecutionTime = 0;

    /**
     * Execute the action, returning whether or not the action has finished executing
     *
     * @param robot The robot to actuate
     * @return Whether or not the action has completed executing
     */
    public abstract boolean executeAction(Robot robot);

    @Override
    public boolean equals(Object action) {
        // Two actions are equal if they do the same thing and begin at the same time.
        return ((Action) action).getClass().equals(this.getClass()) && this.beginExecutionTime == ((Action) action).beginExecutionTime;
    }

    public abstract String getName();
}
