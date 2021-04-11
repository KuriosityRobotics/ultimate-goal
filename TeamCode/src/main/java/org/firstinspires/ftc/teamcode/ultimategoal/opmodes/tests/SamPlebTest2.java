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
public class SamPlebTest2 extends LinearOpMode {
    private  DcMotor intakeTop;
    private  DcMotor intakeBottom;
    private Servo hopper;
    private CRServo leftTurret;
    private CRServo rightTurret;
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private Servo indexer;
    private Servo flap;


    public double pos = 0.22;
    double pos2 = 0.72;
    boolean toggle = false;
    public double flyWheelTargetSpeed = 1750;

    public double yMovement = 0;
    public double xMovement = 0;
    public double turnMovement = 0;

    private final static double MECANUM_POWER_SCALE_FACTOR = 1.414;

    @Override
    public void runOpMode() {
        intakeTop = (DcMotor) hardwareMap.get("intakeTop");
        intakeBottom = (DcMotor) hardwareMap.get("intakeBottom");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTurret = (CRServo) hardwareMap.get("leftTurret");
        rightTurret = (CRServo) hardwareMap.get("rightTurret");
        hopper = (Servo) hardwareMap.get("hopper");
        flap = (Servo) hardwareMap.get("shooterFlap");

        fLeft = (DcMotor) hardwareMap.get("fLeft");
        fRight = (DcMotor) hardwareMap.get("fRight");
        bLeft = (DcMotor) hardwareMap.get("bLeft");
        bRight = (DcMotor) hardwareMap.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);


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

        while (opModeIsActive()) {
            intakeTop.setPower(gamepad2.left_stick_y);
            intakeBottom.setPower(gamepad2.left_stick_y);

            if(gamepad2.a){
//                servo.setPosition(pos);
//                pos -= 0.0001;
                hopper.setPosition(0.0);
            }else if(gamepad2.x){
//                servo.setPosition(pos);
//                pos += 0.0001;
                hopper.setPosition(0.36);
            }

            if(gamepad2.right_stick_x > 0) {
                leftTurret.setDirection(DcMotorSimple.Direction.REVERSE);
                rightTurret.setDirection(DcMotorSimple.Direction.REVERSE);
                leftTurret.setPower(1);
                rightTurret.setPower(1);
            }else if(gamepad2.right_stick_x < 0){
                leftTurret.setDirection(DcMotorSimple.Direction.FORWARD);
                rightTurret.setDirection(DcMotorSimple.Direction.FORWARD);
                leftTurret.setPower(1);
                rightTurret.setPower(1);
            }else{
                leftTurret.setPower(0);
                rightTurret.setPower(0);
            }

            if(gamepad1.dpad_up){
                pos += 0.0001;
            }else if(gamepad1.dpad_down){
                pos -= 0.0001;
            }
            flap.setPosition(pos);

            double yMovement = 0;
            double xMovement = 0;
            double turnMovement = 0;

            yMovement = -gamepad1.left_stick_y;
            xMovement = gamepad1.left_stick_x;
            turnMovement = gamepad1.right_stick_x;

            double fLPower = ((yMovement) + turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);
            double fRPower = ((yMovement) - turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
            double bLPower = ((yMovement) + turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
            double bRPower = ((yMovement) - turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);

            fLeft.setPower(fLPower);
            fRight.setPower(fRPower);
            bLeft.setPower(bLPower);
            bRight.setPower(bRPower);

            if(gamepad1.left_bumper){
                leftFlyWheel.setVelocity(flyWheelTargetSpeed);
                rightFlyWheel.setVelocity(flyWheelTargetSpeed);
            }else{
                leftFlyWheel.setVelocity(0);
                rightFlyWheel.setVelocity(0);
            }


            //pos2 +=gamepad2.left_stick_y*0.02;

            if (SystemClock.elapsedRealtime()-startTime >= 160 && gamepad1.a) {
                toggle = !toggle;
                if(toggle) {
                    pos2 = 0.72;
                }else{
                    pos2 = 0.45;
                }
                startTime = SystemClock.elapsedRealtime();
            }else if(!gamepad1.a){
                pos2 = 0.72;
            }

            indexer.setPosition(pos2);

            telemetry.addLine("left speed: " + Double.toString(leftFlyWheel.getVelocity()));
            telemetry.addLine("right speed: " + Double.toString(rightFlyWheel.getVelocity()));
            telemetry.addLine("servo angle: " + pos);
            telemetry.addLine("servo indexer: " + pos2);
            telemetry.addLine("set speed: " + flyWheelTargetSpeed);

            telemetry.addLine("pos: " + pos);
            telemetry.update();
        }
    }
}
