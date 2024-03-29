package org.firstinspires.ftc.teamcode.ultimategoal.util.auto.actions;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.HopperModule;
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

    int itr = 0;
    boolean completedLastDitch = false;

    @Override
    public boolean executeAction(Robot robot) {
        itr++;

        if (beginExecutionTime == 0) {
            robot.ringManager.autoRaise = true;
            robot.ringManager.autoShootRings = true;

            if (ringsToExpect >= 3) {
                robot.ringManager.autoRaiseThreshold = 2;
            } else {
                robot.ringManager.autoRaiseThreshold = 1;
            }

            robot.shooter.target = this.target;
            robot.shooter.flywheelOn = true;
            robot.shooter.lockTarget = true;

            startingRingsShot = robot.ringManager.getTotalRingsShot();

            beginExecutionTime = robot.getCurrentTimeMilli();
        }

        boolean lastDitch = robot.getCurrentTimeMilli() > beginExecutionTime + 8100;

        int ringsShotSoFar = robot.ringManager.getTotalRingsShot() - startingRingsShot;

//        Log.v("shootstack", "ringsshosofar: " + ringsShotSoFar);
//        Log.v("shootstack", "expect: " + ringsToExpect);
//        Log.v("shootstack", "distsensorpasses: " + robot.ringManager.getDistanceSensorPasses());

        String message = "placeholder";
        if (ringsToExpect >= 3) {
            if (ringsShotSoFar < 2 && robot.ringManager.getForwardDistanceSensorPasses() >= 1 && robot.ringManager.getEntraceSensorReading() < 60) {
                robot.intakeModule.intakePower = 0.8;
                message = "slowing down!";
            } else if (ringsShotSoFar < 2 && robot.ringManager.getForwardDistanceSensorPasses() >= 2) {
                robot.intakeModule.intakePower = -0.2;
                message = "got two!";
            } else if (ringsShotSoFar + robot.ringManager.getDistanceSensorPasses() >= 4 || robot.ringManager.getDistanceSensorPasses() >= 2) {
                robot.intakeModule.intakePower = 0;
                message = "got 4! or we have 2 idk";
            } else if (ringsShotSoFar >= 2) {
                robot.intakeModule.intakePower = 1;
                message = "go ham";
            } else {
                robot.intakeModule.intakePower = 0.85;
                message = "normal ops";
            }

            if (robot.intakeModule.stopIntake || !robot.shooter.isFinishedFiringQueue() || robot.intakeModule.intakePower < 0) {
                robot.drivetrain.setMovements(0, 0, 0);
            } else {
                if (robot.drivetrain.distanceToPoint(end) < 3) {
                    robot.drivetrain.setMovements(0, 0, 0);
                    robot.drivetrain.setBrakePosition(end);
                } else {
                    if (ringsShotSoFar >= 2) {
                        robot.drivetrain.setMovementsTowardsPoint(end, 0.34, 0.5, 0, false, 0);
                    } else {
                        robot.drivetrain.setMovementsTowardsPoint(end, 0.175, 0.3, 0, false, 0);
                    }
                }
            }
        } else if (ringsToExpect >= 1) {
            int systemrings = robot.ringManager.getRingsInShooter() + ringsShotSoFar;
            robot.intakeModule.intakePower = systemrings >= ringsToExpect ? 0 : 1;

            if (robot.intakeModule.stopIntake || !robot.shooter.isFinishedFiringQueue() || robot.intakeModule.intakePower < 0) {
                robot.drivetrain.setMovements(0, 0, 0);
            } else {
                if (robot.drivetrain.distanceToPoint(end) < 3 || ringsShotSoFar >= ringsToExpect) {
                    robot.drivetrain.setMovements(0, 0, 0);
                    robot.drivetrain.setBrakePosition(end);
                } else {
                    robot.drivetrain.setMovementsTowardsPoint(end, 0.2, 0.3, 0, false, 0);
                }
            }
        } else {
            if (robot.drivetrain.distanceToPoint(end) < 3) {
                robot.drivetrain.setMovements(0, 0, 0);
                robot.drivetrain.setBrakePosition(end);
            } else {
                robot.drivetrain.setMovementsTowardsPoint(end, 0.17, 0.1, 0, true, Math.toRadians(-150));
            }
        }

        if (itr % 100 == 0) {
            Log.v("shootstack", message);
        }

//        if (robot.shooter.getTurretVelocity() < 0.01 )


        if (lastDitch && !completedLastDitch) {
            Log.v("shoostack", "last ditching");
            robot.shooter.clearIndexes();
            int tobe = robot.ringManager.getRingsInShooter() + ringsShotSoFar;
            if (robot.shooter.getCurrentHopperPosition() == HopperModule.HopperPosition.LOWERED && !robot.shooter.deliveryQueued() && tobe < ringsToExpect) {
                robot.shooter.deliverRings();
            } else {
                Log.v("shootstack", "didn't need to deliv");
            }
            robot.shooter.queuedIndexes = 3;
            completedLastDitch = true;
        }

        boolean done = (ringsShotSoFar >= ringsToExpect) || (robot.getCurrentTimeMilli() > beginExecutionTime + 9500);

        if (done) {
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
