package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
@Config
public class SamPlebTest2 extends LinearOpMode {
    public FtcDashboard dashboard;

    public TelemetryPacket packet;

    public static double leftIntake = 0.85;
    public static double rightIntake = 0.1;

    public Servo leftBlocker;
    public Servo rightBlocker;

    public static double hopperDown = 0.03;
    public static double hopperUp = 0.55;

    private  DcMotor intakeTop;
    private Servo hopper;

    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private Servo indexer;
    private Servo flap;
    private DcMotorEx turretMotor;

    private Servo wobbleArm1;
    private Servo wobbleArm2;
    private Servo claw;


    public static double pos = 0.2307999;
    double pos2 = 0.72;
    boolean toggle = false;
    public double flyWheelTargetSpeed = 1750;

    public double yMovement = 0;
    public double xMovement = 0;
    public double turnMovement = 0;

    private final static double MECANUM_POWER_SCALE_FACTOR = 1.414;

    @Override
    public void runOpMode() {
        this.dashboard = FtcDashboard.getInstance();
        this.packet = new TelemetryPacket();

        intakeTop = (DcMotor) hardwareMap.get("intakeTop");
        wobbleArm1 = (Servo) hardwareMap.get("wobbleArm1");
        wobbleArm2 = (Servo) hardwareMap.get("wobbleArm2");

        claw = (Servo) hardwareMap.get("wobbleClaw");

        leftBlocker = (Servo) hardwareMap.get("blockerLeft");
        rightBlocker = (Servo) hardwareMap.get("blockerRight");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);

        hopper = (Servo) hardwareMap.get("hopper");
        flap = (Servo) hardwareMap.get("shooterFlap");

        fLeft = (DcMotor) hardwareMap.get("fLeft");
        fRight = (DcMotor) hardwareMap.get("fRight");
        bLeft = (DcMotor) hardwareMap.get("bLeft");
        bRight = (DcMotor) hardwareMap.get("bRight");

        turretMotor = (DcMotorEx) hardwareMap.get("turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel1");
        rightFlyWheel = (DcMotorEx) hardwareMap.get("flyWheel2");
        indexer = (Servo) hardwareMap.get("indexer");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlyWheel.setVelocityPIDFCoefficients(9, 0.4, 0, 11.7);
        rightFlyWheel.setVelocityPIDFCoefficients(9, 0.4, 0, 11.7);

        indexer.setPosition(pos2);



        waitForStart();

        long startTime = 0;

        turretMotor.setTargetPosition(200);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(turretMotor.isBusy()){
            turretMotor.setPower(1);
        }
        turretMotor.setPower(0);

        while (opModeIsActive()) {
            intakeTop.setPower(gamepad2.left_stick_y);
            //0.2232
            if(gamepad2.a){
//                servo.setPosition(pos);
//                pos -= 0.0001;
                hopper.setPosition(hopperDown);
            }else if(gamepad2.x){
//                servo.setPosition(pos);
//                pos += 0.0001;
//                hopper.setPosition(0.36);
                hopper.setPosition(hopperUp);
            }

            if(gamepad2.b){
                wobbleArm1.setPosition(0.2488);
                wobbleArm2.setPosition(0.2488);
                //arm up 0.0059
//arm mid 0.2488
//arm down 0.47466
//claw open 0
//claw close 0.3
            }else if(gamepad2.left_bumper){
                wobbleArm1.setPosition(0);
                wobbleArm2.setPosition(0);
            }else if(gamepad2.right_bumper){
                wobbleArm1.setPosition(0.47);
                wobbleArm2.setPosition(0.47);
            }

            if(gamepad2.right_trigger>0){
                claw.setPosition(0);
            }else if(gamepad2.left_trigger>0){
                claw.setPosition(0.3);
            }

            leftBlocker.setPosition(leftIntake);
            rightBlocker.setPosition(rightIntake);

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
                    pos2 = 0.47;
                }
                startTime = SystemClock.elapsedRealtime();
            }else if(!gamepad1.a){
                pos2 = 0.72;
            }




            indexer.setPosition(pos2);

            telemetry.addLine("left speed: " + leftFlyWheel.getVelocity());
            telemetry.addLine("right speed: " + rightFlyWheel.getVelocity());
            telemetry.addLine("servo angle: " + pos);
            telemetry.addLine("servo indexer: " + pos2);
            telemetry.addLine("set speed: " + flyWheelTargetSpeed);

            telemetry.addLine("pos: " + pos);
            telemetry.update();
        }
    }
}

