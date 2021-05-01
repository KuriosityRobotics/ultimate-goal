package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.vision.Vision;

//@Disabled
@TeleOp
public class VisionTestBitmap extends LinearOpMode {

    public void runOpMode() {
        Vision vision = new Vision(this);

        waitForStart();
        vision.runDetection();
    }
}


