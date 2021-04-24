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
            robot.ringManager.autoRaiseThreshold = 1;

            robot.shooter.target = this.target;
            robot.shooter.flywheelOn = true;
            robot.shooter.lockTarget = true;

            startingRingsShot = robot.ringManager.getTotalRingsShot();

            beginExecutionTime = robot.getCurrentTimeMilli();
        }

        int ringsShotSoFar = robot.ringManager.getTotalRingsShot() - startingRingsShot;

        Log.v("shootstack", "ringsshosofar: " + ringsShotSoFar);
        Log.v("shootstack", "expect: " + ringsToExpect);
        Log.v("shootstack", "distsensorpasses: " + robot.ringManager.getDistanceSensorPasses());

        if (robot.ringManager.getForwardDistanceSensorPasses() >= 4 && !robot.ringManager.getDeliverRings()) {
            robot.intakeModule.intakePower = -0.4;
        } else if (ringsShotSoFar + (robot.ringManager.getDistanceSensorPasses() / 2.0) >= 4) {
            robot.intakeModule.intakePower = 0;
        } else {
            robot.intakeModule.intakePower = 0.9;
        }

//        if (robot.shooter.getTurretVelocity() < 0.01 )
        if (robot.intakeModule.stopIntake || !robot.shooter.isFinishedFiringQueue()) {
            robot.drivetrain.setMovements(0, 0, 0);
        } else {
            if (robot.drivetrain.distanceToPoint(end) < 8) {
                robot.drivetrain.setMovements(0, 0, 0);
                robot.drivetrain.setBrakePosition(end);
            } else {
                robot.drivetrain.setMovementsTowardsPoint(end, 0.2, 0.9, 0, false, 0);
            }
        }

        return (ringsShotSoFar >= ringsToExpect) || (robot.getCurrentTimeMilli() > beginExecutionTime + 8000);
    }

    @Override
    public String getName() {
        return "ShootStackAction";
    }
}
