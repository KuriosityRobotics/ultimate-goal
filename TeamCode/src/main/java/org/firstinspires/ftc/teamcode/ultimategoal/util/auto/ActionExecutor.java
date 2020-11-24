package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

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
     */
    public void updateExecution() {
        for (Action action : executingActions) {
            if (action.executeAction(robot)) {
                executingActions.remove(action);
            }
        }
    }

    /**
     * Registers actions as ready to be executed. They will be executed the next time
     * updateExecution() is run.
     *
     * @param actions An ArrayList of Actions to register.
     * @see #updateExecution()
     */
    public void registerActions(ArrayList<Action> actions) {
        for (Action action : actions) {
            registerAction(action);
        }
    }

    /**
     * Register an action as ready to be executed. They will be executed the next time
     * updateExecution() is run.
     *
     * @param action
     * @see #updateExecution()
     */
    public void registerAction(Action action) {
        // Register the action
        executingActions.add(action);
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
