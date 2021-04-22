package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class pepegamode extends LinearOpMode {
    public void runOpMode() {
        Servo servo = hardwareMap.servo.get("servo");
        AnalogInput input = hardwareMap.get(AnalogInput.class, "servoSensor");

        while (opModeIsActive()) {
            if(gamepad2.a) {
                servo.setPosition(1);
            } else if (gamepad2.b) {
                servo.setPosition(0);
            }

            telemetry.addLine(String.valueOf(input.getVoltage()));

            telemetry.update();
        }
    }
}
