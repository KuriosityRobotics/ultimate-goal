package org.firstinspires.ftc.teamcode.ultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class ShooterCurveFit extends LinearOpMode implements TelemetryProvider {
    Robot robot;

    RealMatrix hypothesis;
    RealMatrix data;

    int currentTestNumber;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);

        telemetry.addLine("This opMode will guide the robot through a process to fit a hypothesis to model what angle to adjust the flap to in order to hit a given target's height and distance.");
        telemetry.addLine("Please ensure the robot is positioned at exactly the back right corner of the field, away from the goals.");
        telemetry.addLine("These are important constants that will be used:");
        telemetry.addLine("Flywheel speed: " + robot.FLY_WHEEL_SPEED);
        telemetry.update();

        waitForStart();

        currentTestNumber = 0;
        while (opModeIsActive() && currentTestNumber < 16) {
            moveToTestPosition();

            guessAim();

            calibrateAim();

            recalculateHypothesis();

            currentTestNumber++;
        }
    }

    /**
     * Move to the next point. The test points are in a grid, starting with (8,8) near the bottom right corner of the field.
     * The opMode must be run with the robot starting at the bottom right corner to ensure an accurate (0,0).
     * 21" increments forward until the shooting line (3 rows), 26" left until roughly the other end of the field (5 columns).
     */
    private void moveToTestPosition() {
        double testPointLocationX = -((26.0 * (currentTestNumber % 5)) + 8);
        double testPointLocationY = ((21.0 * Math.floor(currentTestNumber / 5.0)) + 8);

//        robot.drivetrain.setMovementsToPoint(new Point(testPointLocationX, testPointLocationY));
    }

    /**
     * Takes a guess, using the hypothesis so far, as to how to hit the target.
     */
    private void guessAim() {

    }

    /**
     * Have the user aim the shooter. After the user finalizes the data, the data is added to the
     * matrix and the robot moves on.
     */
    private void calibrateAim() {

    }

    /**
     * Recalculate the hypothesis using the latest data.
     */
    private void recalculateHypothesis() {

    }

    @Override
    public ArrayList<String> getTelemetryData() {
        return null;
    }

    @Override
    public String getName() {
        return "ShooterCurveFit";
    }
}
