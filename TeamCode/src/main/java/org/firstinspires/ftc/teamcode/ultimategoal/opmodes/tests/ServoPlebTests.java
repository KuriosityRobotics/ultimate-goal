package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

@Disabled
@TeleOp
public class ServoPlebTests extends LinearOpMode {
    Servo leftLock;
    Servo rightLock;
    Servo hopperLinkage;
    Servo indexer;

    double leftLockPosition = 1;
    double rightLockPosition = 0;
    double hopperLinkagePosition = 1;
    double indexerPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        leftLock = hardwareMap.servo.get("leftIntakeLock");
        rightLock = hardwareMap.servo.get("rightIntakeLock");
        hopperLinkage = hardwareMap.servo.get("hopperLinkage");
        indexer = hardwareMap.servo.get("indexerServo");

        waitForStart();

        while (opModeIsActive()) {
            leftLockPosition -= 0.0001 * gamepad1.left_stick_y;
//            rightLockPosition -= 0.0001 * gamepad1.right_stick_y;
//            hopperLinkagePosition -= 0.0001 * gamepad1.right_stick_y;
            indexerPosition -= 0.0001 * gamepad1.right_stick_y;

            leftLock.setPosition(leftLockPosition);
//            rightLock.setPosition(rightLockPosition);
//            hopperLinkage.setPosition(hopperLinkagePosition);
            indexer.setPosition(indexerPosition);

            telemetry.addLine("Left lock position: " + leftLockPosition);
            telemetry.addLine("Right lock position: " + rightLockPosition);
            telemetry.addLine("hopper position: " + hopperLinkagePosition);
            telemetry.addLine("indexer position: " + indexerPosition);
            telemetry.update();
        }
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Left lock position: " + leftLockPosition);
        data.add("Right lock position: " + rightLockPosition);
        return data;
    }

    public String getName() {
        return "IntakeTest";
    }
}
