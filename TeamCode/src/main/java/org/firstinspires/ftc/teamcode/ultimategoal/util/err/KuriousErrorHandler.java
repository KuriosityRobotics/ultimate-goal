package org.firstinspires.ftc.teamcode.ultimategoal.util.err;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;

// kurious was the god of error handling in mediaeval norse mythology
public class KuriousErrorHandler implements Thread.UncaughtExceptionHandler {
    LinearOpMode opMode;
    public KuriousErrorHandler(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void uncaughtException(Thread t, Throwable e) {
        Arrays.stream(e.getMessage().split(
                Optional.ofNullable(System.getProperty("line.separator")).orElse("\n")
        )).forEach(opMode.telemetry::addLine);
        opMode.telemetry.update();
        e.printStackTrace();
    }
}
