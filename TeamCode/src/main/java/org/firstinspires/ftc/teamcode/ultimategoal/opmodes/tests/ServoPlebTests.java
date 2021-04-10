package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

//@Disabled
@TeleOp
public class ServoPlebTests extends LinearOpMode {
    Servo hopperLinkage;
    Servo flap;

    double flappos = 0.2;

    @Override
    public void runOpMode() {
        hopperLinkage = hardwareMap.servo.get("hopper");
        flap = hardwareMap.servo.get("shooterFlap");

        waitForStart();

        flap.setPosition(flappos);

        while (opModeIsActive()) {
            flappos += gamepad1.left_stick_y * 0.0001;
            flap.setPosition(flappos);

            telemetry.addData("flap pos: ", flappos);
            telemetry.update();
        }
    }
}
