package org.firstinspires.ftc.teamcode.TeleOp.Meets;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BackLift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.FrontExt;

@TeleOp
public class IntoTheDeepQT extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    FrontExt frontExtension = new FrontExt();
    BackLift backLift = new BackLift();

    ElapsedTime runtime = new ElapsedTime();

    // State variable for cycling wrist positions
    int wristPosition = 0; // 0 = Init, 1 = Middle, 2 = Rotated, 3 = LeftMiddle
    boolean rightStickPressed = false; // Debounce mechanism
    boolean leftStickPressed = false; // Debounce mechanism
    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        frontExtension.init(hardwareMap);
        backLift.init(hardwareMap);
        runtime.reset();
    }

    @Override
    public void loop() {
        // P1 drive code, field centric (up is always up)
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float turn = gamepad1.right_stick_x;

        double mult = gamepad1.left_bumper ? 1 : 0.8;

        if (gamepad1.options) {
            drivetrain.yawReset();
        }

        double botHeading = drivetrain.yawHeading();
        double rotX = right * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = right * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double denim = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        drivetrain.fieldCentricDrive(rotX * mult, rotY * mult, turn * mult, denim / mult);

        /**
         * Button Map:
         * P1:
         * X - Hard Reset
         * A - Transfer Specimen
         * B - Grab Specimen
         * R1 - Pivot Down
         * L1 - Turbo Mode
         * L3 - Wrist Reset to Init
         * R3 - Cycle Wrist Positions (Init -> Middle -> Rotated -> LeftMiddle)
         * D-Pad Up/Down/Right - Slide Positions
         *
         * P2:
         * D-Pad Up/Right/Left - Basket/Specimen Pivot
         * B - Reset BackLift
         */

        // Hard reset all positions
        if (gamepad1.x) {
            backLift.slideClawOpen();
            backLift.slidePivotBase();
            backLift.specimenOpen();
            frontExtension.backPivotBase();
            frontExtension.frontPivotTransfer();
            frontExtension.wristInit();
            frontExtension.frontClawOpen();
            frontExtension.transferIn();
            backLift.slidesBase();
        }

        // Transfer specimen from front to back extension
        if (gamepad1.a) {


            runtime.reset();
            while (runtime.seconds() <= 0.75) {
                frontExtension.frontPivotTransfer();
                frontExtension.backPivotTransfer();
                frontExtension.wristInit();
                frontExtension.transferFullIn();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.1) {
                backLift.slideClawClose();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.125) {
                frontExtension.frontClawOpen();
                frontExtension.frontPivotBase();
                frontExtension.backPivotBase();
            }
        }

        if (gamepad1.y) {
            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                frontExtension.frontClawGrab();
                backLift.slideClawOpen();
            }

            runtime.reset();
            while (runtime.seconds() <= 0.75) {
                frontExtension.frontPivotTransfer();
                frontExtension.backPivotTransfer();
                frontExtension.wristInit();
                frontExtension.transferFullIn();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.1) {
                backLift.slideClawClose();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.125) {
                frontExtension.frontClawOpen();
                frontExtension.frontPivotBase();
                frontExtension.backPivotBase();
            }
        }

        // Lower to grab position
        if (gamepad1.right_bumper) {
            frontExtension.frontPivotGrab();
            frontExtension.frontClawOpen();
        }

        // Grab specimen
        if (gamepad1.b) {
            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                frontExtension.frontClawGrab();
                backLift.slideClawOpen();
            }
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
        }

        // Cycle wrist positions with right stick press
        if (gamepad1.right_stick_button && !rightStickPressed) {
            wristPosition = (wristPosition + 1) % 4; // Cycle between 0, 1, 2, 3
            rightStickPressed = true;

            switch (wristPosition) {
                case 0:
                    frontExtension.wristInit();
                    break;
                case 1:
                    frontExtension.wristMiddle();
                    break;
                case 2:
                    frontExtension.wristRotate();
                    break;
                case 3:
                    frontExtension.wristleftMiddle();
                    break;
            }
        } else if (!gamepad1.right_stick_button) {
            rightStickPressed = false; // Reset debounce flag when button is released
        }


        // Reset wrist to Init position with left stick press
        if (gamepad1.left_stick_button) {
            wristPosition = 0; // Reset state to Init
            frontExtension.wristInit();
        }

        // Slide positions
        if (gamepad1.dpad_up) {
            frontExtension.transferExtend();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        } else if (gamepad1.dpad_right) {
            frontExtension.transferMiddle();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        } else if (gamepad1.dpad_down) {
            frontExtension.transferFullIn();
            frontExtension.frontPivotBase();
            frontExtension.backPivotBase();
            frontExtension.wristInit();
        }



        if (gamepad2.x) {
            runtime.reset();
            while   (runtime.seconds() <= 0.25){
                backLift.slidesBase(); }
            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                backLift.slideClawOpen();
            }
            backLift.slidePivotBase();
        }


        // Basket pivots and specimen handling
        if (gamepad2.dpad_up) {
            backLift.slidesTop();
            backLift.slidePivotDrop();
        } else if (gamepad2.dpad_down) {
            runtime.reset();
            while (runtime.seconds() <= 0.5) {
                backLift.slidesSpecimenHang();
            }
            backLift.specimenOpen();
        } else if (gamepad2.dpad_left) {
            runtime.reset();
            while (runtime.seconds() <= 0.5) {
                backLift.specimenClose();
            }
            backLift.slidesSpecimenPreHang();
        } else if (gamepad2.dpad_right) {
            backLift.slidesMiddle();
            backLift.slidePivotDrop();
        }

        // Reset slides and claws
        if (gamepad2.b) {
            runtime.reset();
            while (runtime.seconds() <= 0.125) {
                backLift.slideClawOpen();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                backLift.slidePivotBase();
            }
            backLift.specimenOpen();
            backLift.slidesBase();
        }

        if (gamepad2.y) {

               backLift.climbTop();
               frontExtension.transferIn();
               frontExtension.frontPivotGrab();
               frontExtension.wristInit();
        }

        if (gamepad2.a) {
            backLift.climbBottom();

        }
    }

    @Override
    public void stop() {
        drivetrain.stopMotors();
    }
}
