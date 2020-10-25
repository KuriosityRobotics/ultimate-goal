package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

import java.util.ArrayList;

public class DrivetrainModule implements Module, TelemetryProvider {
    private Robot robot;
    private boolean isOn;

    // States
    public double yMovement;
    public double xMovement;
    public double turnMovement;

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

    public void init() {
        fLeft = robot.getDcMotor("fLeft");
        fRight = robot.getDcMotor("fRight");
        bLeft = robot.getDcMotor("bLeft");
        bRight = robot.getDcMotor("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    // drivetrain update method applies the powers based on y x and turn movements
    public void update() {
        fLPower = ((yMovement) - turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);
        fRPower = ((yMovement) + turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
        bLPower = ((yMovement) - turnMovement - xMovement * MECANUM_POWER_SCALE_FACTOR);
        bRPower = ((yMovement) + turnMovement + xMovement * MECANUM_POWER_SCALE_FACTOR);

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
        if (Math.abs(power) < 0.01) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }
    }

    private void setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        fLeft.setZeroPowerBehavior(zeroPowerBehavior);
        fRight.setZeroPowerBehavior(zeroPowerBehavior);
        bLeft.setZeroPowerBehavior(zeroPowerBehavior);
        bRight.setZeroPowerBehavior(zeroPowerBehavior);
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
