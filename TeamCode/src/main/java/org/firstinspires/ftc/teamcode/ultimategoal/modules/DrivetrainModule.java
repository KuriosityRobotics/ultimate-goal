package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ultimategoal.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;

import java.util.ArrayList;

public class DrivetrainModule implements Module, TelemetryProvider {
    private Robot robot;
    private boolean isOn;

    // States
    public double yMovement = 0;
    public double xMovement = 0;
    public double turnMovement = 0;

    // Constants
    private final static double POWER_SCALE_FACTOR = 0.8;
    private final static double MECANUM_POWER_SCALE_FACTOR = 1.414;

    // Motors
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    public DrivetrainModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    public void initModule() {
        fLeft = robot.getDcMotor("fLeft");
        fRight = robot.getDcMotor("fRight");
        bLeft = robot.getDcMotor("bLeft");
        bRight = robot.getDcMotor("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // drivetrain update method applies the powers based on y x and turn movements
    public void update() {
        double fLPower = ((yMovement) + turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);
        double fRPower = ((yMovement) - turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
        double bLPower = ((yMovement) + turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
        double bRPower = ((yMovement) - turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);


        double maxPower = Math.abs(fLPower);
        if (Math.abs(fRPower) > maxPower) {
            maxPower = Math.abs(fRPower);
        }
        if (Math.abs(bLPower) > maxPower) {
            maxPower = Math.abs(bLPower);
        }
        if (Math.abs(bRPower) > maxPower) {
            maxPower = Math.abs(bRPower);
        }
        double scaleDown = 1.0;
        if (maxPower > 1.0) {
            scaleDown = 1.0 / maxPower;
        }

        fLPower *= scaleDown;
        fRPower *= scaleDown;
        bLPower *= scaleDown;
        bRPower *= scaleDown;

        fLPower *= POWER_SCALE_FACTOR;
        fRPower *= POWER_SCALE_FACTOR;
        bLPower *= POWER_SCALE_FACTOR;
        bRPower *= POWER_SCALE_FACTOR;

        setMotorPowers(fLPower, fRPower, bLPower, bRPower);
    }

    public void fileDump() {

    }

    public void setMovements(double xMovement, double yMovement, double turnMovement) {
        this.xMovement = xMovement;
        this.yMovement = yMovement;
        this.turnMovement = turnMovement;
    }

    private void setMotorPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        setMotorPower(fLeft, fLPower);
        setMotorPower(fRight, fRPower);
        setMotorPower(bLeft, bLPower);
        setMotorPower(bRight, bRPower);
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.06) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    public boolean isOn() {
        return isOn;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("xMovement: " + xMovement);
        data.add("yMovement: " + yMovement);
        data.add("turnMovement: " + turnMovement);
        return data;
    }

    public String getName() {
        return "DrivetrainModule";
    }
}
