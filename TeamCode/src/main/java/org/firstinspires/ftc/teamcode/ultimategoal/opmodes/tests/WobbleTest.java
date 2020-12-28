package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class WobbleTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx wobbleMotor = (DcMotorEx) hardwareMap.get("wobbleMotor");

        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.a) {
                wobbleMotor.setTargetPosition(-400);
                while (wobbleMotor.isBusy()) {
                    wobbleMotor.setPower(0.5);
                }
                wobbleMotor.setPower(0);
            }
        }
    }
}
