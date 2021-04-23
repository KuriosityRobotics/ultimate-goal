package org.firstinspires.ftc.teamcode.ultimategoal.util.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDumpProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.closestPointOnLineToPoint;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.lineSegmentCircleIntersection;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.lineSegmentPointDistance;

@Config
public class PathFollow implements TelemetryProvider, FileDumpProvider {
    Robot robot;
    private final boolean isFileDump = false;

    Point clippedRobotPosition = new Point(0, 0);
    Point targetPoint = new Point(0, 0);

    // constants
    public static final double DISTANCE_THRESHOLD = 2;
    public static final double ANGLE_THRESHOLD = Math.toRadians(2);
    public static final double FOLLOW_RADIUS = 15;
    public static final double SLIP_FACTOR = 0;
    public static double SLOWDOWN_P = 0.055;
    public static double SLOWDOWN_CONSTANT = 0.1;

    // states
    private boolean isTargetingLastPoint;
    private final String description;
    private final Waypoint[] path;
    private int pathIndex;
    private boolean registeredLastAction;

    // settings
    private double direction = 0;
    private double angleLockHeadingAtEnd = 0;
    private boolean willAngleLockAtEnd = false;

    public PathFollow(Waypoint[] path, Robot robot, String description) {
        this.path = path;
        this.robot = robot;
        this.description = description;

        robot.telemetryDump.registerProvider(this);
    }

    public void followPath(double direction, double moveSpeed, double turnSpeed, boolean willAngleLockAtEnd, double angleLockHeadingAtEnd) {
        followPath(direction, moveSpeed, turnSpeed, willAngleLockAtEnd, angleLockHeadingAtEnd, true);
    }

    public void followPath(double direction, double moveSpeed, double turnSpeed, boolean willAngleLockAtEnd, double angleLockHeadingAtEnd, boolean brakeAtEnd) {
        this.pathIndex = 0; // Reset pathIndex
        this.registeredLastAction = false;
        this.direction = direction;
        this.willAngleLockAtEnd = willAngleLockAtEnd;
        this.angleLockHeadingAtEnd = angleLockHeadingAtEnd;
        this.isTargetingLastPoint = false;

        robot.actionExecutor.registerActions(path[0].actions);

        while (robot.isOpModeActive()) {
            Point robotPosition = robot.drivetrain.getCurrentPosition();
            double robotHeading = robot.drivetrain.getCurrentHeading();

            clippedRobotPosition = calculateClippedPosition(robotPosition);
            targetPoint = calculateTargetPoint(clippedRobotPosition);

            if (robot.actionExecutor.requiresStop()) { // an action requires us to stop
                robot.drivetrain.setMovements(0, 0, 0);
            } else if (isDoneMoving(robotPosition, robotHeading)) { // We're at the end of the path
                robot.drivetrain.brakePoint = path[path.length - 1];

                if (willAngleLockAtEnd) {
                    robot.drivetrain.brakeHeading = angleLockHeadingAtEnd;
                }

                robot.drivetrain.setMovements(0, 0, 0);

                if (!registeredLastAction) {
                    robot.actionExecutor.registerActions(path[path.length - 1].actions);
                    registeredLastAction = true;
                }

                if (robot.actionExecutor.isDoneExecutingQueue()) {
                    return;
                }
            } else { // We still need to move
                if (isTargetingLastPoint && brakeAtEnd) {
                    // use brake
                    robot.drivetrain.setMovements(0, 0, 0);

                    robot.drivetrain.brakePoint = path[path.length - 1];

                    if (willAngleLockAtEnd) {
                        robot.drivetrain.brakeHeading = angleLockHeadingAtEnd;
                    }
                } else {
                    // go to the target point
                    double pathDistanceLeft = 0;
                    for (int i = pathIndex; i < path.length - 2; i++) {
                        Point start = path[i];
                        Point end = path[i+1];

                        pathDistanceLeft += Math.hypot(start.x - end.x, start.y - end.y);
                    }

//                    double scaledMoveSpeed = Math.min(moveSpeed, (pathDistanceLeft * SLOWDOWN_P) + SLOWDOWN_CONSTANT);
                    double scaledMoveSpeed = moveSpeed;

                    robot.drivetrain.setMovementsTowardsPoint(targetPoint, scaledMoveSpeed, turnSpeed, direction, false, angleLockHeadingAtEnd);
                }
            }

            robot.actionExecutor.updateExecution();
        }
    }

    public String getFileData() {
        return Arrays.toString(path);
    }

    public String getFileName() {
        return description + ".path";
    }

    /**
     * Clips the robot's current position onto the path, also updating the pathIndex to reflect
     * where the robot is along the path.
     *
     * @param center
     * @return clipped position of the robot onto the path
     */
    private Point calculateClippedPosition(Point center) {
        Point clipped = new Point();

        double nearestClipDist = Double.MAX_VALUE;
        int clippedIndex = pathIndex;

        // only checks the current line and the next line (no skipping)
        for (int i = pathIndex; i < Math.min(path.length - 1, pathIndex + 2); i++) {
            Point start = path[i];
            Point end = path[i + 1];

            double thisClipDist = lineSegmentPointDistance(center, start, end);

            // if this clip distance is the lowest, remember this point, where it is on the path, and distance
            if (thisClipDist < nearestClipDist) {
                nearestClipDist = thisClipDist;
                clipped = closestPointOnLineToPoint(center, start, end);
                clippedIndex = i;
            }
        }

        // Update our progress along the path using where we actually are
        if (clippedIndex != pathIndex) {
            for (int skippedIndex = pathIndex; skippedIndex <= clippedIndex; skippedIndex++) {
                robot.actionExecutor.registerActions(path[skippedIndex].actions);
            }
        }

        pathIndex = clippedIndex;

        return clipped;
    }

    /**
     * Calculate what point the robot should move towards.
     *
     * @param center
     * @return The point the robot should move towards.
     */
    private Point calculateTargetPoint(Point center) {
        Point followPoint = new Point();

        Point pathSegmentStartPoint = path[pathIndex];
        double distToSegmentStart = Math.hypot(center.x - pathSegmentStartPoint.x, center.y - pathSegmentStartPoint.y);

        // If we're close enough to the end of the path, just try to go to the end of the path
        if (Math.hypot(center.x - path[path.length - 1].x, center.y - path[path.length - 1].y) < 36 && pathIndex == path.length - 2) {
            followPoint = path[path.length - 1];
            isTargetingLastPoint = true;
        } else {
            // only look at lines on current index or next index
            for (int i = pathIndex; i < Math.min(path.length - 1, pathIndex + 2); i++) {
                Point start = path[i];
                Point end = path[i + 1];

                ArrayList<Point> intersections = lineSegmentCircleIntersection(center, FOLLOW_RADIUS, start, end);

                for (Point thisIntersection : intersections) {
                    double thisDistanceToSegmentStart = Math.hypot(thisIntersection.x - pathSegmentStartPoint.x, thisIntersection.y - pathSegmentStartPoint.y);

                    if (thisDistanceToSegmentStart > distToSegmentStart) { // If this intersection is furthest along the segment of the path
                        followPoint = thisIntersection; // Use this point
                    }
                }
            }
        }

        return followPoint;
    }

    private Point adjustTargetPoint(Point targetPoint) {
        double robotHeading = robot.drivetrain.getCurrentHeading();

        double robotSlipX = SLIP_FACTOR * robot.drivetrain.getOdometryXVel();
        double robotSlipY = SLIP_FACTOR * robot.drivetrain.getOdometryYVel();

        double slipX = robotSlipX * Math.cos(robotHeading) + robotSlipY * Math.sin(robotHeading);
        double slipY = robotSlipY * Math.cos(robotHeading) - robotSlipX * Math.sin(robotHeading);

        return new Point(targetPoint.x - slipX, targetPoint.y - slipY);
    }

    private boolean isDoneMoving(Point robotPosition, double heading) {
        Point endPoint = path[path.length - 1];

        return (Math.hypot(robotPosition.x - endPoint.x, robotPosition.y - endPoint.y) < DISTANCE_THRESHOLD)
                && (!willAngleLockAtEnd || Math.abs(angleWrap(angleLockHeadingAtEnd - heading)) < ANGLE_THRESHOLD)
                && pathIndex == path.length - 2;
    }

    public boolean isFileDump() {
        return isFileDump;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("path: " + description);
        data.add("clippedX: " + clippedRobotPosition.x);
        data.add("clippedY: " + clippedRobotPosition.y);
        data.add("targetX: " + targetPoint.x);
        data.add("targetY: " + targetPoint.y);
        data.add("pathIndex: " + pathIndex);
        return data;
    }

    public String getName() {
        return "PathFollower" + description;
    }
}