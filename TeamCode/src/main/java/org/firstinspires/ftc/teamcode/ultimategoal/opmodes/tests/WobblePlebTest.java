package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class WobblePlebTest extends LinearOpMode{
    public Servo leftServo;
    public Servo rightServo;
    public DcMotorEx intakeTop;
    public DcMotorEx intakeBottom;

    public DistanceSensor distance;

    @Override
    public void runOpMode() {
        intakeTop = (DcMotorEx) hardwareMap.get("intakeTop");
        intakeBottom = (DcMotorEx) hardwareMap.get("intakeBottom");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

        intakeTop.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeBottom.setDirection(DcMotorSimple.Direction.FORWARD);

//        intakeTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        leftServo = (Servo) hardwareMap.get("rightIntakeLock");
        rightServo = (Servo) hardwareMap.get("leftIntakeLock");
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        waitForStart();

        double left = 0.5;
        double right = 0.5;


        double counter = 0;
        while (opModeIsActive()) {
            //close
            //0.80932
            //0.20824

            //open
            //0.61842
            //0.36688

            left += gamepad1.left_stick_y*0.001;
            right += gamepad1.right_stick_y*0.001;

            leftServo.setPosition(left);
            rightServo.setPosition(right);

            telemetry.addLine("LEFT " + left);
            telemetry.addLine("RIGHT " + right);
            telemetry.update();
            if(distance.getDistance(DistanceUnit.MM) <= 57){
                counter++;
            }else{
                counter = 0;
            }
            if(counter >= 5){
                intakeTop.setPower(1);
                intakeBottom.setPower(1);
            }else {
                intakeTop.setPower(-1);
                intakeBottom.setPower(-1);
            }



//            if(gamepad1.a){
//                rings = 0;
//                leftServo.setPosition(0.15);
//                rightServo.setPosition(0.85);
//                Log.d("INTAKE", "pressed");
//            }else if(gamepad1.b){
//                leftServo.setPosition(0.78);
//                rightServo.setPosition(0.22);
//            }
        }
    }
}
