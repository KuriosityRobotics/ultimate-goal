package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class SamPlebTest extends LinearOpMode{
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;

    public double pos = 0.75;
    double pos2 = 0.8;

    public double flyWheelTargetSpeed = 1550;

    @Override
    public void runOpMode() {

        leftFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel1");
        rightFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel2");
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            flyWheelTargetSpeed += gamepad1.left_stick_y*10;

            leftFlyWheel.setVelocity(flyWheelTargetSpeed);
            rightFlyWheel.setVelocity(-flyWheelTargetSpeed);
            pos += gamepad1.right_stick_y*0.0001;
            //pos2 +=gamepad2.left_stick_y*0.02;
            if(gamepad2.a){
                pos2 = 0.8;
            }else if(gamepad2.b){
                pos2 = 0.98;
            }


            telemetry.addLine("left speed: " + Double.toString(leftFlyWheel.getVelocity()));
            telemetry.addLine("right speed: " + Double.toString(rightFlyWheel.getVelocity()));
            telemetry.addLine("servo angle: " +pos);
            telemetry.addLine("servo indexer: " +pos2);
            telemetry.addLine("set speed: " + flyWheelTargetSpeed);
            telemetry.update();
        }
    }
}
