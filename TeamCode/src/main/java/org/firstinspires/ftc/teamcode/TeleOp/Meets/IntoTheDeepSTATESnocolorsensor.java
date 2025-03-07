package org.firstinspires.ftc.teamcode.TeleOp.Meets;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BackLift;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.FrontExt;

@TeleOp
public class IntoTheDeepSTATESnocolorsensor extends OpMode {
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

        telemetry.addData("Status", "Initialized");
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


        /* code ideas:
        add color sensor to claw to determine if claw actively has something in grasp- make it auto go back if it confirms to have something
        make cycles less time based and more state based
        when a is pressed it brings the calw all the way in instead of full cycles, then when gamepad 2 is used to lift slides it finishes the transfer system
        add software limits to maximize sizing
        add reset button for driver 2

         */

        // Hard reset all positions
        if (gamepad1.x) {
            backLift.slideClawOpen();
            backLift.transfergrab();

            frontExtension.backPivotTransfer();
            frontExtension.frontPivotGrab();
            frontExtension.wristInit();
            frontExtension.frontClawOpen();
            frontExtension.transferIn();
            backLift.slidesBase();
        }

        // Lower to grab position
        if (gamepad1.right_bumper) {
            wristPosition= 0;
            frontExtension.frontPivotGrab();
            frontExtension.frontClawOpen();
            frontExtension.backPivotBase();
        }


        // Grab specimen
        if (gamepad1.b) {
            runtime.reset(); // Reset the timer
            while (runtime.seconds() <= .25) { frontExtension.frontClawGrab(); // Grab the sample
            }

            frontExtension.frontPivotGrab();
            frontExtension.backPivotTransfer();
                frontExtension.frontPivotTransfer();
                frontExtension.transferFullIn();
                frontExtension.wristInit();


        }

// Cycle wrist positions with right stick press
        if (gamepad1.right_stick_button && !rightStickPressed) {
            wristPosition = (wristPosition + 1) % 4; // Cycle between 0, 1, 2, 3
            rightStickPressed = true;

            // Update wrist position based on the new value
            switch (wristPosition) {
                case 0:
                    frontExtension.wristInit();
                    break;
                case 1:
                    frontExtension.wristleftMiddle();
                    break;
                case 2:
                    frontExtension.wristRotate();
                    break;
                case 3:
                    frontExtension.wristMiddle();
                    break;
            }
        } else if (!gamepad1.right_stick_button) {
            rightStickPressed = false; // Reset debounce flag when button is released
        }

// Cycle wrist positions backward with left stick press
        if (gamepad1.left_stick_button && !leftStickPressed) {
            wristPosition = (wristPosition - 1 + 4) % 4; // Cycle between 0, 1, 2, 3 (handle negative values)
            leftStickPressed = true;

            // Update wrist position based on the new value
            switch (wristPosition) {
                case 0:
                    frontExtension.wristInit();
                    break;
                case 1:
                    frontExtension.wristleftMiddle();
                    break;
                case 2:
                    frontExtension.wristRotate();
                    break;
                case 3:
                    frontExtension.wristMiddle();
                    break;
            }
        } else if (!gamepad1.left_stick_button) {
            leftStickPressed = false; // Reset debounce flag when button is released
        }

        // Slide positions
        if (gamepad1.dpad_up) {
            frontExtension.transferExtend();
            frontExtension.frontPivotGrab();
            frontExtension.backPivotPreGrab();
            frontExtension.wristInit();
            wristPosition= 0;
        } else if (gamepad1.dpad_right) {
            frontExtension.transferMiddle();
            frontExtension.frontPivotGrab();
            frontExtension.backPivotPreGrab();
            frontExtension.wristInit();
            wristPosition= 0;
        } else if (gamepad1.dpad_down) {
            frontExtension.transferFullIn();
            frontExtension.frontPivotGrab();
            frontExtension.backPivotPreGrab();
            frontExtension.wristInit();
            wristPosition= 0;
        }



        if (gamepad2.x) {
            backLift.slidesBase();
        }


        // Basket pivots and specimen handling
        if (gamepad2.dpad_up) {
            runtime.reset();
            while (runtime.seconds() <= 0.2) {
                backLift.slideClawClose();
                frontExtension.frontClawOpen();
                frontExtension.frontPivotBase();
            }
            backLift.slidesTop();
           backLift.transferdrop();



        } else if (gamepad2.dpad_right) {
            runtime.reset();
            while (runtime.seconds() <= .2) {
                backLift.slideClawClose();
                frontExtension.frontClawOpen();
                frontExtension.frontPivotBase();
            }
            backLift.slidesMiddle();
            backLift.transferdrop();
        }

        // Reset slides and claws
        if (gamepad2.b) {
            runtime.reset();
            while (runtime.seconds() <= 0.125) {
                backLift.slideClawOpen();
            }
            runtime.reset();
            while (runtime.seconds() <= 0.25) {
                backLift.transfergrab();
            }
        }

        if (gamepad2.y) {

               backLift.climbTop();
              backLift.transfergrab();
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
