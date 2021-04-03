package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class SamPlebTest extends LinearOpMode {
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private Servo indexer;

    public double pos = 0.75;
    double pos2 = 0.68;
    boolean toggle = false;
    public double flyWheelTargetSpeed = 1700;

    @Override
    public void runOpMode() {

        leftFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel1");
        rightFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel2");
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer = (Servo) hardwareMap.get("indexer");
//        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFlyWheel.setVelocityPIDFCoefficients(9, 0.4, 0, 11.7);
        rightFlyWheel.setVelocityPIDFCoefficients(9, 0.4, 0, 11.7);

        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setPosition(pos2);
        waitForStart();

        long startTime = 0;
        long startTime2 = 0;

        while (opModeIsActive()) {

            flyWheelTargetSpeed += gamepad1.left_stick_y * 10;

            leftFlyWheel.setVelocity(flyWheelTargetSpeed);
            rightFlyWheel.setVelocity(flyWheelTargetSpeed);

            //pos2 +=gamepad2.left_stick_y*0.02;

            if (SystemClock.elapsedRealtime()-startTime >= 200 && gamepad1.a) {
                toggle = !toggle;
                if(toggle) {
                    pos2 = 0.68;
                }else{
                    pos2 = 0.3;
                }
                startTime = SystemClock.elapsedRealtime();
            }else if(!gamepad1.a){
                pos2 = 0.68;
            }

            indexer.setPosition(pos2);


            telemetry.addLine("left speed: " + Double.toString(leftFlyWheel.getVelocity()));
            telemetry.addLine("right speed: " + Double.toString(rightFlyWheel.getVelocity()));
            telemetry.addLine("servo angle: " + pos);
            telemetry.addLine("servo indexer: " + pos2);
            telemetry.addLine("set speed: " + flyWheelTargetSpeed);
            telemetry.update();
        }
    }
}
