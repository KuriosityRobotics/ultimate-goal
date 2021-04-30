package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Target;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Action;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Point;

/**
 * An action where the robot moves towards the end point slowly, intaking and shooting all rings.
 */
public class ShootStackAction extends Action {
    Point end;
    Target.ITarget target;
    int ringsToExpect;

    int startingRingsShot;

    public ShootStackAction(int ringsToExpect, Point end, Target.ITarget target) {
        this.ringsToExpect = ringsToExpect;
        this.end = end;
        this.target = target;
        this.stopMovementForExecution = true;
    }

    @Override
    public boolean executeAction(Robot robot) {
        if (beginExecutionTime == 0) {
            robot.ringManager.autoRaise = true;
            robot.ringManager.autoShootRings = true;
            robot.ringManager.autoRaiseThreshold = 2;
            robot.ringManager.intakeFollowThrough = false;

            robot.shooter.target = this.target;
            robot.shooter.flywheelOn = true;
            robot.shooter.lockTarget = true;

            startingRingsShot = robot.ringManager.getTotalRingsShot();

            beginExecutionTime = robot.getCurrentTimeMilli();
        }

        int ringsShotSoFar = robot.ringManager.getTotalRingsShot() - startingRingsShot;

        if (ringsShotSoFar >= 2) {
            robot.ringManager.intakeFollowThrough = true;
        }

        Log.v("shootstack", "ringsshosofar: " + ringsShotSoFar);
        Log.v("shootstack", "expect: " + ringsToExpect);
        Log.v("shootstack", "distsensorpasses: " + robot.ringManager.getDistanceSensorPasses());

        if (robot.ringManager.getForwardDistanceSensorPasses() >= 2 && ringsShotSoFar < 2) {
            robot.intakeModule.intakePower = -0.4;
        } else if (ringsShotSoFar + robot.ringManager.getDistanceSensorPasses() >= 4 || robot.ringManager.getDistanceSensorPasses() >= 2) {
            robot.intakeModule.intakePower = 0;
        } else if (ringsShotSoFar >= 2) {
            robot.intakeModule.intakePower = 1;
        } else {
            robot.intakeModule.intakePower = 0.9;
        }

//        if (robot.shooter.getTurretVelocity() < 0.01 )
        if (robot.intakeModule.stopIntake || !robot.shooter.isFinishedFiringQueue() || robot.intakeModule.intakePower < 0) {
            robot.drivetrain.setMovements(0, 0, 0);
        } else {
            if (robot.drivetrain.distanceToPoint(end) < 3) {
                robot.drivetrain.setMovements(0, 0, 0);
                robot.drivetrain.setBrakePosition(end);
            } else {
                if (ringsShotSoFar >= 2) {
                    robot.drivetrain.setMovementsTowardsPoint(end, 0.25, 0.9, 0, false, 0);
                } else {
                    robot.drivetrain.setMovementsTowardsPoint(end, 0.145, 0.9, 0, false, 0);
                }
            }
        }

        boolean done = (ringsShotSoFar >= ringsToExpect) || (robot.getCurrentTimeMilli() > beginExecutionTime + 9500);

        if (done) {
            robot.ringManager.intakeFollowThrough = true;
            robot.intakeModule.intakePower = 0;
            robot.ringManager.autoRaiseThreshold = 1;
            robot.ringManager.resetRingCounters();
        }

        return done;
    }

    @Override
    public String getName() {
        return "ShootStackAction";
    }
}
