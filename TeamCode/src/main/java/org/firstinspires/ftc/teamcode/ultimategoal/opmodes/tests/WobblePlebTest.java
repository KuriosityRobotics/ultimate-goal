package org.firstinspires.ftc.teamcode.ultimategoal.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class WobblePlebTest extends LinearOpMode {


    public DcMotor intakeTop;
    public DcMotor intakeBottom;

    public AnalogInput distance;

    @Override
    public void runOpMode() {
        intakeTop = (DcMotor) hardwareMap.get("intakeTop");
        intakeBottom = (DcMotor) hardwareMap.get("intakeBottom");

        intakeTop.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBottom.setDirection(DcMotorSimple.Direction.REVERSE);


        distance = hardwareMap.get(AnalogInput.class, "distance");


//        intakeTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        waitForStart();

        double left = 0.5;
        double right = 0.5;


        double counter = 0;
        boolean seenRing = false;
        int passes = 0;

        double timeLastPass = time;

        while (opModeIsActive()) {


            intakeTop.setPower(gamepad2.left_stick_y);
            intakeBottom.setPower(gamepad2.left_stick_y);

            //close
            //0.80932
            //0.20824

            //open
            //0.61842
            //0.36688


            double voltage = distance.getVoltage();
            telemetry.addLine("dist sensor voltage: " + Double.toString(distance.getVoltage()));
            telemetry.addLine("total passes of ring: " + passes);

            if (Math.abs(2.2 - voltage) < Math.abs(1.1 - voltage)) {
                if (!seenRing) {
                    seenRing = true;
                }

                telemetry.addLine("________________________\n" +
                        "< something is inside me >\n" +
                        " ------------------------\n" +
                        "        \\   ^__^\n" +
                        "         \\  (oo)\\_______\n" +
                        "            (__)\\       )\\/\\\n" +
                        "                ||----w |\n" +
                        "                ||     ||\n");
            } else {

                if (seenRing) {
                    if (gamepad2.left_stick_y > 0)
                        passes = passes + 1;
                    else
                        passes = passes - 1;
                    seenRing = false;
                }
                telemetry.addLine(" ________________________\n" +
                        "< something is not inside me >\n" +
                        " ------------------------\n" +
                        "        \\   ^__^\n" +
                        "         \\  (oo)\\_______\n" +
                        "            (__)\\       )\\/\\\n" +
                        "                ||----w |\n" +
                        "                ||     ||\n");
            }


            if (time - timeLastPass > 0.2 && (passes & 1) == 1 && seenRing) {
                passes++;
            }
            timeLastPass = time;
            telemetry.update();
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
