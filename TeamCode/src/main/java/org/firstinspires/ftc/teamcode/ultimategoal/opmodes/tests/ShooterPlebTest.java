package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class ShooterPlebTest extends LinearOpMode {
    DcMotorEx flyWheel1;
    DcMotorEx flyWheel2;

    double flyWheelSpeed = 1550;

    @Override
    public void runOpMode() throws InterruptedException {
        flyWheel1 = (DcMotorEx) hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = (DcMotorEx) hardwareMap.dcMotor.get("flyWheel2");

        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            flyWheelSpeed -= gamepad1.left_stick_y * 1;

            flyWheel1.setVelocity(flyWheelSpeed);
            flyWheel2.setVelocity(flyWheelSpeed);

            telemetry.addLine("target: " + flyWheelSpeed);
            telemetry.addLine("flyWheel1 speed: " + flyWheel1.getVelocity());
            telemetry.addLine("flyWheel2 speed: " + flyWheel2.getVelocity());
            telemetry.update();
        }
    }
}
