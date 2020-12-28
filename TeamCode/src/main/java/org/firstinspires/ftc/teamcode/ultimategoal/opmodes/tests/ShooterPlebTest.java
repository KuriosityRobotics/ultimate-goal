package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ShooterPlebTest extends LinearOpMode {
    DcMotorEx flyWheel1;
    DcMotorEx flyWheel2;
    Servo shooterFlap;
    Servo indexerServo;
    Servo hopperLinkage;
    private static final double INDEX_OPEN_POSITION = 0.385;
    private static final double INDEX_PUSH_POSITION = 0.125;
    private static final double HOPPER_RAISED_POSITION = 0.965;
    double flyWheelSpeed = 1900;

    @Override
    public void runOpMode() throws InterruptedException {
        flyWheel1 = (DcMotorEx) hardwareMap.dcMotor.get("flyWheel1");
        flyWheel2 = (DcMotorEx) hardwareMap.dcMotor.get("flyWheel2");
        shooterFlap = (Servo) hardwareMap.get("shooterFlap");
        indexerServo = (Servo) hardwareMap.get("indexerServo");
        hopperLinkage = (Servo) hardwareMap.get("hopperLinkage");

        flyWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel1.setVelocityPIDFCoefficients(8.5, 0.6, 0, 11.7);
        flyWheel2.setVelocityPIDFCoefficients(8.5, 0.6, 0, 11.7);

        waitForStart();
        double pos = 0.63;
        while (opModeIsActive()) {
            hopperLinkage.setPosition(HOPPER_RAISED_POSITION);
            flyWheelSpeed += gamepad1.left_stick_y * 1;

            flyWheel1.setVelocity(flyWheelSpeed);
            flyWheel2.setVelocity(flyWheelSpeed);
            pos += gamepad1.right_stick_y * 0.005;
            if (gamepad1.a) {
                indexerServo.setPosition(INDEX_PUSH_POSITION);
            } else if (gamepad1.b) {
                indexerServo.setPosition(INDEX_OPEN_POSITION);
            }
            shooterFlap.setPosition(pos);
            telemetry.addLine("target: " + flyWheelSpeed);
            telemetry.addLine("flyWheel1 speed: " + flyWheel1.getVelocity());
            telemetry.addLine("flyWheel2 speed: " + flyWheel2.getVelocity());
            telemetry.update();
        }
    }
}
