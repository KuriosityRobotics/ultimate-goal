package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

@Disabled
@TeleOp
public class DrivetrainPlebTest extends LinearOpMode {
    DcMotor fLeft;
    DcMotor fRight;
    DcMotor bLeft;
    DcMotor bRight;

    Toggle g1a = new Toggle();

    int motorIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        waitForStart();

        while (opModeIsActive()) {
            if (g1a.isToggled(gamepad1.a)) {
                motorIndex++;
                if (motorIndex > 3) {
                    motorIndex = 0;
                }
            }

            switch (motorIndex) {
                case 0:
                    fLeft.setPower(1);
                    fRight.setPower(0);
                    bLeft.setPower(0);
                    bRight.setPower(0);
                    break;
                case 1:
                    fLeft.setPower(0);
                    fRight.setPower(1);
                    bLeft.setPower(0);
                    bRight.setPower(0);
                    break;
                case 2:
                    fLeft.setPower(0);
                    fRight.setPower(0);
                    bLeft.setPower(1);
                    bRight.setPower(0);
                    break;
                case 3:
                    fLeft.setPower(0);
                    fRight.setPower(0);
                    bLeft.setPower(0);
                    bRight.setPower(1);
                    break;
            }

            telemetry.addLine("Motor index: " + motorIndex);
            telemetry.update();
        }
    }
}
