package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class IntakeLocks extends LinearOpMode {
    Servo leftIntakeLock;
    Servo rightIntakeLock;

    public void runOpMode() {
        leftIntakeLock = hardwareMap.servo.get("leftIntakeLock");
        rightIntakeLock = hardwareMap.servo.get("rightIntakeLock");

        waitForStart();

        double position = 0.35;

        while (opModeIsActive()) {
            position += gamepad1.right_stick_y * 0.0001;

            leftIntakeLock.setPosition(position);
            rightIntakeLock.setPosition(1 - position);

            telemetry.addLine("position: " + position);
            telemetry.update();
        }
    }
}
