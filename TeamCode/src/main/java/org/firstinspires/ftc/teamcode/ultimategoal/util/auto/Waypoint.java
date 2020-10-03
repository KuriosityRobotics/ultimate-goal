package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions.ActionExecutor;

import java.util.ArrayList;

public class Waypoint extends Point {
    ArrayList<Action> actions;

    public Waypoint(double x, double y) {
        super(x, y);
        this.actions = new ArrayList<>();
    }

    public Waypoint(double x, double y, Action action) {
        super(x, y);
        this.actions = new ArrayList<>();
        actions.add(action);
    }

    public Waypoint(double x, double y, ArrayList<Action> actions) {
        super(x, y);
        this.actions = actions;
    }

    public void registerActions(ActionExecutor actionExecutor) {
        actionExecutor.registerActions(actions);
    }

    public Point toPoint() {
        return new Point(x, y);
    }
}
