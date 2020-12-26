package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.Iterator;

public class ActionExecutor implements TelemetryProvider {
    private Robot robot;
    private ArrayList<Action> executingActions;

    public ActionExecutor(Robot robot) {
        robot.telemetryDump.registerProvider(this);

        executingActions = new ArrayList<>();

        this.robot = robot;
    }

    /**
     * Update execution on all actions registered as currently executing.
     *
     * @param pathFollow The PathFollow object to modify if there are any PathFollowActions.
     */
    public void updateExecution(PathFollow pathFollow) {
        Iterator<Action> actionsItr = executingActions.iterator();
        while (actionsItr.hasNext()) {
            Action nextAction = actionsItr.next();

            if (nextAction instanceof PathFollowAction) {
                if (((PathFollowAction) nextAction).executeAction(pathFollow)) {
                    actionsItr.remove();
                }
            } else if (nextAction instanceof RobotAction) {
                if (((RobotAction) nextAction).executeAction(robot)) {
                    actionsItr.remove();
                }
            } else {
                throw new Error("Unknown action: " + nextAction.getName() + ": " + nextAction);
            }
        }
    }

    public boolean isDoneExecutingQueue() {
        return executingActions.isEmpty();
    }

    /**
     * Registers actions as ready to be executed. They will be executed the next time
     * updateExecution() is run.
     *
     * @param actions An ArrayList of Actions to register.
     * @see #updateExecution(PathFollow)
     */
    public void registerActions(ArrayList<Action> actions) {
        if (actions != null) {
            for (Action action : actions) {
                registerAction(action);
            }
        }
    }

    /**
     * Register an action as ready to be executed. They will be executed the next time
     * updateExecution() is run.
     *
     * @param action
     */
    public void registerAction(Action action) {
        // Register the action
        executingActions.add(action);
    }

    public boolean requiresStop() {
        for (Action action : executingActions) {
            if (action.stopMovementForExecution) {
                return true;
            }
        }

        return false;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        String executingActions = "Actions being executed: ";

        for (int i = 0; i < this.executingActions.size(); i++) {
            executingActions = executingActions + this.executingActions.get(i).getName();

            if (i != this.executingActions.size()) {
                executingActions = executingActions + ", ";
            }
        }

        data.add(executingActions);

        return data;
    }

    public String getName() {
        return "ActionExecutor";
    }
}
