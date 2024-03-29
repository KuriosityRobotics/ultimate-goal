package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;

public class Waypoint extends Point {
    ArrayList<Action> actions;

    public Waypoint(double x, double y) {
        super(x, y);
        this.actions = new ArrayList<>();
    }

    public Waypoint(Point point) {
        super(point.x, point.y);
    }

    public Waypoint(Point point, Action action) {
        super(point.x, point.y);
        this.actions = new ArrayList<>();
        actions.add(action);
    }

    public Waypoint(Point point, ArrayList<Action> actions) {
        super(point.x, point.y);
        this.actions = actions;
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
