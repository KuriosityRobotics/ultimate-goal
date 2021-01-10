package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.opmodes.BlueAuto;

import java.lang.reflect.InvocationTargetException;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode {
    private static T265Camera slamra = null;
    Robot robot;
    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        telemetry.addLine("X: " + translation.getX());
        telemetry.addLine("Y: " + translation.getY());
        telemetry.addLine("ROTATION: " + Math.toDegrees(rotation.getRadians()));
        telemetry.update();
    }

    @Override
    public void stop() {
        slamra.stop();
        try {
            slamra.getClass().getMethod("cleanup").setAccessible(true);
            slamra.getClass().getMethod("cleanup").invoke(slamra);
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException ex) {}
    }

}
