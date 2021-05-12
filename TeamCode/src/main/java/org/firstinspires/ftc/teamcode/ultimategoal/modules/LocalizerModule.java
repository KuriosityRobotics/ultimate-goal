package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.math.Matrix;
import org.opencv.core.Mat;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.add;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.angleWrap;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.determinant3x3;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.inverse3x3;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.multiply;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.negate;
import static org.firstinspires.ftc.teamcode.ultimategoal.util.math.MathFunctions.transpose;

public class LocalizerModule extends ModuleCollection implements TelemetryProvider {

    private Robot robot;
    private boolean isOn;

    public double x = 0;
    public double y = 0;
    public double heading = 0;
    public Matrix covariance = new Matrix(new double[][]{
            {2, 0, 0},
            {0, 2, 0},
            {0, 0, Math.toRadians(3)}
    }); // some random number for now.... its how sure it is initially

    private final VuforiaModule vuforiaModule;
    private final OdometryModule odometryModule;

    public LocalizerModule(Robot robot, boolean isOn) {
        this.robot = robot;
        this.isOn = isOn;

        vuforiaModule = new VuforiaModule(robot, isOn);
        odometryModule = new OdometryModule(robot, isOn);

        modules = new Module[]{odometryModule, vuforiaModule};

        robot.telemetryDump.registerProvider(this);
    }

    @Override
    public void update() {
        if (odometryModule.isOn()) odometryModule.update();
        if (vuforiaModule.isOn()) vuforiaModule.update();

        // Kalman Filter

        // STEP 1: prediction
        double odoDX = odometryModule.dRobotX;
        double odoDY = odometryModule.dRobotY;
        double odoDTheta = odometryModule.dTheta;

        double predX = x + odoDX * Math.cos(heading) + odoDY * Math.sin(heading);
        double predY = y - odoDX * Math.sin(heading) + odoDY * Math.cos(heading);
        double predHeading = heading + odoDTheta;

        // jacobian of states with respect to states
        Matrix G = new Matrix(new double[][]{
                {1, 0, -odoDX * Math.sin(heading) + odoDY * Math.cos(heading)},
                {0, 1, -odoDX * Math.cos(heading) - odoDY * Math.sin(heading)},
                {0, 0, 1}
        }); // just learned how stupid the matrix class i made is

        // jacobian of states with respect to controls
        Matrix V = new Matrix(new double[][]{
                {Math.cos(heading), Math.sin(heading), 0},
                {-Math.sin(heading), Math.cos(heading), 0},
                {0, 0, 1}
        });

        Matrix M = new Matrix(new double[][]{
                {0.03 * odoDX, 0, 0},
                {0, 0.03 * odoDY, 0},
                {0, 0, 0.0005 * odoDTheta}
        }); // plugged in random values for now, would want to make acceleration (slip) based, this is basically error per update
        // also make more dynamic

        Matrix predCovariance = add(
                multiply(G, multiply(covariance, transpose(G))),
                multiply(V, multiply(M, transpose(V)))
        );

        // STEP 2: observation
        if (vuforiaModule.visibleTracker ){
            VuforiaTrackable tracker = vuforiaModule.tracker;
            double tX = (double)tracker.getLocation().getTranslation().get(0);
            double tY = (double)tracker.getLocation().getTranslation().get(1);
            double tPhi = (double)tracker.getLocation().getTranslation().get(2);

            double predTRX = (tX - predX) * Math.cos(heading) - (tY - predY) * Math.sin(heading);
            double predTRY = (tX - predX) * Math.sin(heading) + (tY - predY) * Math.cos(heading);
            double predTRPhi = tPhi - heading;

            // jacobian of observation with respect to states
            Matrix H = new Matrix(new double[][]{
                    {-Math.cos(heading), Math.sin(heading), -(tX-predX)*Math.sin(heading) - (tY-predY)*Math.cos(heading)},
                    {-Math.sin(heading), -Math.cos(heading), (tX-predX)*Math.cos(heading) - (tY-predY)*Math.sin(heading)},
                    {0,0,-1}
            });

            Matrix Q = new Matrix(new double[][]{
                    {0.01, 0, 0},
                    {0, 0.01, 0},
                    {0, 0, 0.00030462}
            }); // pretty random values right now, entries are std of observations squared

            Matrix S = add(
                    multiply(H, multiply(predCovariance, transpose(H))),
                    Q
            );

            // STEP 3: updating prediction based off observation
            if (determinant3x3(S) != 0 || inverse3x3(S) != null){ // hit the dip if S is non-invertible, both conditions are same btw

                // Kalman Gain
                Matrix K = multiply(predCovariance, multiply(transpose(H), inverse3x3(S)));

                Matrix z = new Matrix(new double[][]{
                        {vuforiaModule.tRX},
                        {vuforiaModule.tRY},
                        {vuforiaModule.tRPhi}
                });

                Matrix zPred = new Matrix(new double[][]{
                        {predTRX},
                        {predTRY},
                        {predTRPhi}
                });

                // correction is based of predicted observation error and Kalman Gain
                Matrix correction = multiply(K, add(z, negate(zPred)));
                double correctionX = correction.getCell(0,0);
                double correctionY = correction.getCell(1, 0);
                double correctionHeading = correction.getCell(2, 0);

                predX += correctionX;
                predY += correctionY;
                predHeading += correctionHeading;
                predCovariance = multiply(
                        add(
                                new Matrix(new double[][]{
                                        {1,0,0},
                                        {0,1,0},
                                        {0,0,1}
                                }),

                                negate(multiply(K,H))
                        ),

                        predCovariance
                );
            }
        }

        x = predX;
        y = predY;
        heading = angleWrap(predHeading);
        covariance = predCovariance;

    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("X: " + x);
        data.add("Y: " + y);
        data.add("Heading: " + Math.toDegrees(heading));

        data.add("Variance X: " + covariance.getCell(0,0));
        data.add("Variance Y: " + covariance.getCell(1,1));
        data.add("Variance Heading: " + covariance.getCell(2,2));

        return data;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "Localizer Module";
    }
}
