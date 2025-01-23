package org.firstinspires.ftc.teamcode.TeleOp.Drives;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

@TeleOp
public class PolarDriveTest extends OpMode {

    Drivetrain drivetrain = new Drivetrain();
    @Override
    public void init(){
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float turn = gamepad1.right_stick_x;

        if (gamepad1.options) {
            drivetrain.yawReset();
        }

        double botHeading = drivetrain.yawHeading();

        double rotX = right * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = right * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double denim = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        drivetrain.polarDrive(rotX, rotY, turn, denim);
    }
    @Override
    public void stop() {
        drivetrain.stopMotors();
    }
}
