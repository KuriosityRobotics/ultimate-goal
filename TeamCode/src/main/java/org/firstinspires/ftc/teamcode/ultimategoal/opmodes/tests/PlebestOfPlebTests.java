package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class PlebestOfPlebTests extends LinearOpMode {
    int i;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Loop count", i);
            telemetry.update();
            sleep(5);

            i++;
        }
    }
}