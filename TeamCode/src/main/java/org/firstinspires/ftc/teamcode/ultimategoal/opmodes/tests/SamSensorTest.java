package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Disabled
@TeleOp
public class SamSensorTest extends LinearOpMode {
    public AnalogInput distance;

    @Override
    public void runOpMode() {
        distance = hardwareMap.analogInput.get("distance");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("distance " + getDistance());
            telemetry.update();
        }
    }

    public double getDistance() {
        double voltage_temp_average = 0;

        for (int i = 0; i < 5; i++) {
            voltage_temp_average += distance.getVoltage();

        }
        voltage_temp_average /= 5;

        //33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
        return (33.9 + -69.5 * (voltage_temp_average) + 62.3 * Math.pow(voltage_temp_average, 2) + -25.4 * Math.pow(voltage_temp_average, 3) + 3.83 * Math.pow(voltage_temp_average, 4)) * 10;
    }
}
