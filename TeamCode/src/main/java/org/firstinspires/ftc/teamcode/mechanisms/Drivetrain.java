package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    public void init(HardwareMap hwMap) {
        //call Motor names
        //TODO: Motor Names
        frontLeftMotor = hwMap.get(DcMotor.class, "leftFront"); //motor 3
        frontRightMotor = hwMap.get(DcMotor.class, "rightFront"); //motor 2
        backLeftMotor = hwMap.get(DcMotor.class, "leftBack"); //motor 1
        backRightMotor = hwMap.get(DcMotor.class, "rightBack"); //motor 0
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

        //run without encoder, stop encoders RR (Implement later)
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //No coasting!
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void yawReset() {
        imu.resetYaw();
    }

    public double yawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    private void setPowers(double frontLeftPower, double frontRightPower,
                           double backLeftPower, double backRightPower) {
        double maxSpeed = 1;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void drive(float fw, float r, float turn, double multiplier) {
        //fw = forward, r = right, turn = rotate
        double frontLeftPower = ((fw + r) + turn) * multiplier;
        double frontRightPower = ((fw - r) - turn) * multiplier;
        double backLeftPower = ((fw - r) + turn) * multiplier;
        double backRightPower = ((fw + r) - turn) * multiplier;
        setPowers(frontLeftPower, frontRightPower,
                backLeftPower, backRightPower);
    }

    public void polarDrive(double theta, double power, double turn, double mult) {
        if (mult > 1) {
            mult -= 1;
        }
        //mult should be between 0.5 and 1
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = (power * cos/max + turn);
        double frontRightPower = (power * sin/max - turn);
        double backLeftPower = (power * sin/max + turn);
        double backRightPower = (power * cos/max - turn);

        if ((power + Math.abs(turn)) > mult) {
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void fieldCentricDrive(double rotX, double rotY, double rx, double denominator) {
        double frontLeftPower = (rotY + rotX + rx) / (denominator);
        double backLeftPower = (rotY - rotX + rx) / (denominator);
        double frontRightPower = (rotY - rotX - rx) / (denominator);
        double backRightPower = (rotY + rotX - rx) / (denominator);


        setPowers(frontLeftPower, frontRightPower,
                backLeftPower, backRightPower);
    }
}