package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

public abstract class Action {
    protected boolean stopMovementForExecution = false;
    protected long beginExecutionTime = 0;

    @Override
    public boolean equals(Object action) {
        // Two actions are equal if they do the same thing and begin at the same time.
        return ((RobotAction) action).getClass().equals(this.getClass()) && this.beginExecutionTime == ((RobotAction) action).beginExecutionTime;
    }

    public abstract String getName();

    public String toString() {
        return this.getName() + beginExecutionTime;
    }
}
