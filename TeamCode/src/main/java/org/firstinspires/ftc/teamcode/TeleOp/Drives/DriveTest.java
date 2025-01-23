package org.firstinspires.ftc.teamcode.TeleOp.Drives;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

@TeleOp
public class DriveTest extends OpMode {

    Drivetrain drivetrain = new Drivetrain();
    @Override
    public void init(){
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop(){
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float turn = gamepad1.right_stick_x;
        double multiplier;

        if (gamepad1.right_bumper) {
            multiplier = 1;
        } else {
            multiplier = 0.5;
        }
        drivetrain.drive(forward, right, turn, multiplier);
    }

    @Override
    public void stop() {
        drivetrain.stopMotors();
    }
}
