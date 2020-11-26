package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ultimategoal.util.Toggle;

@Disabled
@TeleOp
public class ShooterMotorIdentification extends LinearOpMode {
    DcMotor flyWheel1;
    DcMotor flyWheel2;

    Toggle g1a = new Toggle();

    int motorIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        flyWheel1 = hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = hardwareMap.dcMotor.get("flyWheel2");

        waitForStart();

        while (opModeIsActive()) {
            if (g1a.isToggled(gamepad1.a)) {
                motorIndex++;
                if (motorIndex > 1) {
                    motorIndex = 0;
                }
            }

            switch (motorIndex) {
                case 0:
                    flyWheel1.setPower(1);
                    flyWheel2.setPower(0);
                    break;
                case 1:
                    flyWheel1.setPower(0);
                    flyWheel2.setPower(1);
                    break;
            }

            telemetry.addLine("currently running: " + motorIndex);
            telemetry.addLine("0 position: " + flyWheel1.getCurrentPosition());
            telemetry.addLine("1 position: " + flyWheel2.getCurrentPosition());
            telemetry.update();
        }
    }
}
