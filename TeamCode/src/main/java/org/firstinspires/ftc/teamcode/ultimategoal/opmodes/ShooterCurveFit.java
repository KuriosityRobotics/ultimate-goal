package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.ToggleButton;
import org.firstinspires.ftc.teamcode.ultimategoal.util.auto.Point;
import org.firstinspires.ftc.teamcode.ultimategoal.util.shooter.TowerGoal;

import java.util.ArrayList;

public class ShooterCurveFit extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    final static TowerGoal TARGET_GOAL = TowerGoal.RED_HIGH;

    // two columns: distance height
    RealMatrix data;

    // one column: optimalAngle
    RealMatrix angles;

    // five columns/parameters: distance distance^2 height height^2 distance*height
    RealMatrix hypothesis;

    int currentTestNumber;

    ToggleButton a = new ToggleButton();

    double flapPosition;

    String status;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        telemetry.addLine("This opMode will guide the robot through a process mainly to gather data.");
        telemetry.addLine("Throughout the process, the robot will attempt to develop a model. This is not the only possible model and is only a benchline.");
        telemetry.addLine("Please ensure the robot is positioned at exactly the back left corner of the field, away from the goals.");
        telemetry.addLine("The robot will aim towards the following goal: " + TARGET_GOAL.name());
        telemetry.addLine("These are important constants that will be used:");
        telemetry.addLine("Flywheel speed: " + robot.FLY_WHEEL_SPEED);
        telemetry.update();

        waitForStart();

        telemetry.clear();

        robot.startModules();

        boolean exit = false;
        while (opModeIsActive()) {
            exit = moveManuallyToTestPosition();

            if (exit) {
                break;
            }

            guessAim();

            calibrateAim();

            recalculateHypothesis();

            currentTestNumber++;
        }

        // TODO: Filedumping shenanigans
    }

    /**
     * Move to the next point. The test points are in a grid, starting with (8,8) near the bottom left (blue left) corner of the field.
     * The opMode must be run with the robot starting at the bottom left corner to ensure an accurate (0,0).
     * 21" increments forward until the shooting line (3 rows), 26" right until roughly the other end of the field (5 columns).
     */
    private void moveToTestPosition() {
        double testPointLocationX = (26.0 * (currentTestNumber % 5)) + 8;
        double testPointLocationY = (21.0 * Math.floor(currentTestNumber / 5.0)) + 8;

        Point testPoint = new Point(testPointLocationX, testPointLocationY);

        status = "Moving to point: " + testPoint;

        robot.drivetrain.moveToPoint(testPoint);

        status = "";

        turnToGoal();
    }

    private boolean moveManuallyToTestPosition() {
        status = "Move the robot to the next desired testing position. Use the joysticks as you would normally drive. Press 'a' to continue, or 'b' to end this program and save all data.";

        while (!gamepad1.a) {
            if (gamepad1.b) {
                return true;
            }

            double yMovement = 0;
            double xMovement = 0;
            double turnMovement = 0;

            yMovement = -gamepad1.left_stick_y;
            xMovement = gamepad1.left_stick_x;
            turnMovement = gamepad1.right_stick_x;

            robot.drivetrain.setMovements(xMovement, yMovement, turnMovement);
        }

        status = "";

        turnToGoal();

        return false;
    }

    private void turnToGoal() {
        status = "Turning to goal...";

        robot.drivetrain.turnTo(robot.aimBot.headingToTarget(TARGET_GOAL));
    }

    /**
     * Takes a guess, using the hypothesis so far, as to how to hit the target.
     */
    private void guessAim() {
        RealMatrix currentScenarioData = new Array2DRowRealMatrix(
                generateRowOfData(robot.aimBot.distanceToTarget(TARGET_GOAL), robot.aimBot.towerGoalHeight(TARGET_GOAL))
        );

        status = "Calculating estimated flap angle...";
        RealMatrix output = currentScenarioData.multiply(hypothesis);
        status = "";

        flapPosition = output.getEntry(0, 0);
    }

    private double[][] generateRowOfData(double distance, double height) {
        return new double[][]{
                {distance, distance * distance, height, height * height, distance * height}
        };
    }

    /**
     * Have the user aim the shooter. After the user finalizes the data, the data is added to the
     * matrix and the robot moves on.
     */
    private void calibrateAim() {
        status = "Please change the flap angle using the left joystick on gamepad1 to the optimal angle. Press 'a' to index. After achieving the optimal angle, press 'y'.";

        while (!gamepad1.y) {
            flapPosition -= gamepad1.right_stick_y * 0.000001;

            if (flapPosition > 1) {
                flapPosition = 1;
            } else if (flapPosition < 0) {
                flapPosition = 0;
            }

            robot.shooterModule.shooterFlapPosition = flapPosition;
            robot.shooterModule.flyWheelTargetSpeed = robot.FLY_WHEEL_SPEED;

            if (a.isToggled(gamepad1.a)) {
                robot.shooterModule.indexRing = true;
            }
        }

        robot.shooterModule.flyWheelTargetSpeed = 0;

        status = "Recording data...";

        addData(robot.aimBot.distanceToTarget(TARGET_GOAL), robot.aimBot.towerGoalHeight(TARGET_GOAL));

        status = "";
    }

    private void addData(double distance, double height) {
        double[][] existingData = data.getData();
        double[][] newRow = generateRowOfData(distance, height);

        double[][] newData = new double[existingData.length + newRow.length][];

        System.arraycopy(existingData, 0, newData, 0, existingData.length);
        System.arraycopy(newRow, 0, newData, existingData.length, newRow.length);

        data = new Array2DRowRealMatrix(newData);
    }

    /**
     * Recalculate the hypothesis using the latest data.
     */
    private void recalculateHypothesis() {
        status = "Recalculating hypothesis...";

        // Temporarily using the normal equation.
        hypothesis = (new SingularValueDecomposition(data.transpose().multiply(data))).getSolver().getInverse().multiply(data.transpose().multiply(angles));

        status = "";
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add(status);
        data.add("Current parameters (no regularization): " + hypothesis);
        return data;
    }

    @Override
    public String getName() {
        return "ShooterCurveFit";
    }
}
