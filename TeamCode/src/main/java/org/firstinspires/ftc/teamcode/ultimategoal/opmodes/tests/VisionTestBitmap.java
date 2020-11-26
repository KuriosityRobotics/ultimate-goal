package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;


import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.HopperModule;
import org.firstinspires.ftc.teamcode.ultimategoal.modules.WobbleModule;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.Vision;

import java.util.ArrayList;

@TeleOp
public class VisionTestBitmap extends LinearOpMode {

    public void runOpMode() {
        Vision vision = new Vision(this);

        waitForStart();
        vision.runDetection();
    }
}


