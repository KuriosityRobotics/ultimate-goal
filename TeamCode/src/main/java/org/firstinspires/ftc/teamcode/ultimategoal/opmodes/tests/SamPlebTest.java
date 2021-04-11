package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class SamPlebTest extends LinearOpMode {
    private Servo wobbleArm1;
    private Servo wobbleArm2;
    private Servo claw;
    private Servo blockerLeft;
    private Servo blockerRight;


    public double armPos = 0.1;
    double blockerLeftPos = 0.85727;
    double blockerRightPos = 0;
    double clawPos = 0.5;
    boolean toggle = false;
    public double flyWheelTargetSpeed = 1750;

    @Override
    public void runOpMode() {

        wobbleArm1 = (Servo) hardwareMap.get("wobbleArm1");
        wobbleArm2 = (Servo) hardwareMap.get("wobbleArm2");

        blockerLeft = (Servo) hardwareMap.get("blockerLeft");
        blockerRight = (Servo) hardwareMap.get("blockerRight");
        claw = (Servo) hardwareMap.get("wobbleClaw");

        waitForStart();

        long startTime = 0;
        long startTime2 = 0;

        while (opModeIsActive()) {
            armPos +=gamepad1.left_stick_y*0.001;
            wobbleArm1.setPosition(armPos);
            wobbleArm2.setPosition(armPos);

            clawPos +=gamepad1.right_stick_y*0.001;
            claw.setPosition(clawPos);


            //left 0 0.85727
            //right 0 0.0

            //left 90 0.492
            //right 90 0.347

            //arm up 0.0059
            //arm mid 0.2488
            //arm down 0.47466

            //claw open 0
            //claw close 0.3
            telemetry.addLine("arm pos: " + armPos);
            telemetry.addLine("left pos: " + blockerLeftPos);
            telemetry.addLine("right pos: " + blockerRightPos);
            telemetry.addLine("claw pos: " + clawPos);
            telemetry.update();
        }
    }
}
