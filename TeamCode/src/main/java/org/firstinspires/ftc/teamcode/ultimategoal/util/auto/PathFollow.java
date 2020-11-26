package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDumpProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.closestPointOnLineToPoint;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.lineSegmentCircleIntersection;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.auto.MathFunctions.lineSegmentPointDistance;

public class PathFollow implements TelemetryProvider, FileDumpProvider {
    Robot robot;
    private boolean isFileDump = false;

    Point clippedPoint = new Point(0, 0);
    Point targetPoint = new Point(0, 0);
    Point adjustedTargetPoint;

    // constants
    public static final double DISTANCE_THRESHOLD = 0.75;
    public static final double ANGLE_THRESHOLD = Math.toRadians(2);
    public static final double FOLLOW_RADIUS = 15;
    public static final double SLIP_FACTOR = 0;

    // states
    private boolean isTargetingLastPoint = false;
    private String description;
    private Waypoint[] path;
    private int pathIndex = 0;
    private boolean registeredLastAction = false;

    // settings
    private double direction = 0;
    private double angleLockHeading = 0;
    private boolean willAngleLock = false;

    public PathFollow(Waypoint[] path, Robot robot, String description) {
        this.path = path;
        this.robot = robot;
        this.description = description;

        robot.telemetryDump.registerProvider(this);
    }

    public void pathFollow(double direction, double moveSpeed, double turnSpeed, boolean willAngleLock, double angleLockHeading) {
        pathIndex = 0; // Reset pathIndex
        registeredLastAction = false;
        this.direction = direction;
        this.willAngleLock = willAngleLock;
        this.angleLockHeading = angleLockHeading;
        isTargetingLastPoint = false;

        robot.actionExecutor.registerActions(path[0].actions);

        while (robot.isOpModeActive()) {
            Point robotPoint = robot.drivetrain.getCurrentPosition();
            double robotHeading = robot.drivetrain.getCurrentHeading();

            clippedPoint = clipToPath(path, robotPoint);
            targetPoint = findTarget(path, clippedPoint, robotHeading);
            adjustedTargetPoint = adjustTargetPoint(targetPoint);

            robot.drivetrain.setMovementsToPoint(adjustedTargetPoint, moveSpeed, turnSpeed, direction, willAngleLock, angleLockHeading, isTargetingLastPoint, FOLLOW_RADIUS);

            robot.actionExecutor.updateExecution();

            boolean isDoneMoving = isDoneMoving(path, robotPoint, robotHeading);
            if (isDoneMoving && !registeredLastAction) {
                robot.actionExecutor.registerActions(path[path.length - 1].actions);
                registeredLastAction = true;
            } else if (isDoneMoving && robot.actionExecutor.isDoneExecutingQueue()) {
                robot.drivetrain.setMovements(0, 0, 0);

                robot.drivetrain.brakePoint = path[path.length - 1];

                if (willAngleLock) {
                    robot.drivetrain.brakeHeading = angleLockHeading;
                }

                return;
            }
        }
    }

    public String getFileData() {
        return Arrays.toString(path);
    }

    public String getFileName() {
        return description + ".path";
    }

    private Point clipToPath(Waypoint[] path, Point center) {
        Point clipped = new Point();

        double nearestClipDist = Double.MAX_VALUE;
        int clippedIndex = pathIndex;

        // only checks the current line and the next line (no skipping)
        for (int i = pathIndex; i < Math.min(path.length - 1, pathIndex + 2); i++) {
            Point start = path[i];
            Point end = path[i + 1];

            double thisClipDist = lineSegmentPointDistance(center, start, end);

            // if this clip distance is record low set the clip point to the clip point set the clippedIndex to index so later we can update the index we are at
            if (thisClipDist < nearestClipDist) {
                nearestClipDist = thisClipDist;
                clipped = closestPointOnLineToPoint(center, start, end);
                clippedIndex = i;
            }
        }

        if (clippedIndex != pathIndex) {
            for (int skippedIndex = pathIndex; skippedIndex <= clippedIndex; skippedIndex++) {
                robot.actionExecutor.registerActions(path[skippedIndex].actions);
            }
        }

        pathIndex = clippedIndex;

        return clipped;
    }

    private Point findTarget(Waypoint[] path, Point center, double heading) {
        Point followPoint = new Point();

        Point lineStartPoint = path[pathIndex];
        double distToFirst = Math.hypot(center.x - lineStartPoint.x, center.y - lineStartPoint.y);

        // only look at lines on current index or next index
        for (int i = pathIndex; i < Math.min(path.length - 1, pathIndex + 2); i++) {
            Point start = path[i];
            Point end = path[i + 1];

            ArrayList<Point> intersections = lineSegmentCircleIntersection(center, FOLLOW_RADIUS, start, end);

            double nearestAngle = Double.MAX_VALUE;

            for (Point thisIntersection : intersections) {

                double angle = Math.atan2(thisIntersection.x - center.x, thisIntersection.y - center.y) + direction;
                double deltaAngle = Math.abs(angleWrap(angle - heading));
                double thisDistToFirst = Math.hypot(thisIntersection.x - lineStartPoint.x, thisIntersection.y - lineStartPoint.y);

                if (deltaAngle < nearestAngle && thisDistToFirst > distToFirst) {
                    nearestAngle = deltaAngle;
                    followPoint.x = thisIntersection.x;
                    followPoint.y = thisIntersection.y;
                }
            }
        }

        if (Math.hypot(center.x - path[path.length - 1].x, center.y - path[path.length - 1].y) < FOLLOW_RADIUS * 1.5 && pathIndex == path.length - 2) {
            followPoint = path[path.length - 1];
            isTargetingLastPoint = true;
        }

        return followPoint;
    }

    private Point adjustTargetPoint(Point targetPoint) {
        double robotHeading = robot.drivetrain.getCurrentHeading();

        double robotSlipX = SLIP_FACTOR * robot.drivetrain.velocityModule.getxVel();
        double robotSlipY = SLIP_FACTOR * robot.drivetrain.velocityModule.getyVel();

        double slipX = robotSlipX * Math.cos(robotHeading) + robotSlipY * Math.sin(robotHeading);
        double slipY = robotSlipY * Math.cos(robotHeading) - robotSlipX * Math.sin(robotHeading);

        return new Point(targetPoint.x - slipX, targetPoint.y - slipY);
    }

    private boolean isDoneMoving(Waypoint[] path, Point center, double heading) {
        Point endPoint = path[path.length - 1];

        return (Math.hypot(center.x - endPoint.x, center.y - endPoint.y) < DISTANCE_THRESHOLD) && (!willAngleLock || Math.abs(angleWrap(angleLockHeading - heading)) < ANGLE_THRESHOLD) && pathIndex == path.length - 2;
    }

    public boolean isFileDump() {
        return isFileDump;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("path: " + description);
        data.add("clippedX: " + clippedPoint.x);
        data.add("clippedY: " + clippedPoint.y);
        data.add("targetX: " + targetPoint.x);
        data.add("targetY: " + targetPoint.y);
        data.add("pathIndex: " + pathIndex);
        return data;
    }

    public String getName() {
        return "PathFollower" + description;
    }
}