package org.firstinspires.ftc.teamcode.TeleOp.Meets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@Autonomous(name = "5+0 RIGHTSTATES", group = "Autonomous")
public class FIVEPLUSZERORIGHTSTATES extends LinearOpMode {

    public static class Slides {
        private final DcMotorEx slidesL;
        private final DcMotorEx slidesR;

        public Slides(HardwareMap hwMap) {
            slidesL = hwMap.get(DcMotorEx.class, "slidesL");
            slidesR = hwMap.get(DcMotorEx.class, "slidesR");
            slidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidesL.setDirection(DcMotorSimple.Direction.FORWARD);
            slidesR.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        public class SlidesUp implements Action {
            private boolean slidesInit = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!slidesInit) {
                    slidesL.setPower(1);
                    slidesR.setPower(1);
                    slidesInit = true;
                }

                double posL = slidesL.getCurrentPosition();
                packet.put("slideLPos", posL);

                double posR = slidesR.getCurrentPosition();
                packet.put("slideRPos", posR);

                if (posL < 1850 & posR < 1850) {
                    return true;
                } else {
                    slidesL.setPower(0.05);
                    slidesR.setPower(0.05);
                    return false;
                }
            }
        }

        public Action slidesUp() {
            return new SlidesUp();
        }

        public class SlidesDown implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    slidesL.setPower(-1);
                    slidesR.setPower(-1);
                    init = true;
                }

                double posL = slidesL.getCurrentPosition();
                packet.put("slideLPos", posL);

                double posR = slidesR.getCurrentPosition();
                packet.put("slideRPos", posR);

                if (posL > 15 & posR > 15) {
                    return true;
                } else {
                    slidesL.setPower(-.3);
                    slidesR.setPower(-.3);
                    return false;
                }
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }



        public class SlidesSpec implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    slidesL.setPower(-1);
                    slidesR.setPower(-1);
                    init = true;
                }

                double posL = slidesL.getCurrentPosition();
                packet.put("slideLPos", posL);

                double posR = slidesR.getCurrentPosition();
                packet.put("slideRPos", posR);

                if (posL > 1400 & posR > 1400) {
                    return true;
                } else {
                    slidesL.setPower(-0);
                    slidesR.setPower(-0);
                    return false;
                }
            }
        }


        public Action slidesSpec() {
            return new SlidesSpec();
        }

    }


    public static class ExtFront {
        private final Servo backPivot;
        private final Servo frontPivot;
        private final Servo wristClaw;
        private final Servo frontClaw;
        private final Servo leftTransfer;
        private final Servo rightTransfer;

        public ExtFront(HardwareMap hwMap) {
            leftTransfer = hwMap.get(Servo.class, "left");
            rightTransfer = hwMap.get(Servo.class, "right");
            frontClaw = hwMap.get(Servo.class, "frontclaw");
            wristClaw = hwMap.get(Servo.class, "wrist");
            frontPivot = hwMap.get(Servo.class, "frontpivot");
            backPivot = hwMap.get(Servo.class, "backpivot");

            //front claw system
            rightTransfer.setDirection(Servo.Direction.REVERSE);
            frontClaw.setDirection(Servo.Direction.REVERSE);
            frontPivot.setDirection(Servo.Direction.REVERSE);
            backPivot.setDirection(Servo.Direction.REVERSE);
        }


        public class TransferIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftTransfer.setPosition(.65);
                rightTransfer.setPosition(.65);
                return false;
            }
        }

        public Action transferIn() {
            return new TransferIn();
        }

        public class TransferExtend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftTransfer.setPosition(1);
                rightTransfer.setPosition(1);
                return false;
            }
        }

        public Action transferExtend() {
            return new TransferExtend();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontClaw.setPosition(0.65);
                return false;
            }
        }

        public Action clawOpen() {
            return new ClawOpen();
        }

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontClaw.setPosition(.26);
                return false;
            }
        }

        public Action clawClose() {
            return new ClawClose();
        }

        public class WristInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristClaw.setPosition(0.43);
                return false;
            }
        }

        public Action wristInit() {
            return new WristInit();
        }

        public class WristRotate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristClaw.setPosition(0.245);
                return false;
            }
        }

        public Action wristRotate() {
            return new WristRotate();
        }


        public class FrontPivotGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontPivot.setPosition(.95);
                return false;
            }
        }

        public Action frontPivotGrab() {
            return new FrontPivotGrab();
        }

        public class FrontPivotTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                frontPivot.setPosition(0.2);
                return false;
            }
        }

        public Action frontPivotTransfer() {
            return new FrontPivotTransfer();
        }

        public class BackPivotBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backPivot.setPosition(0.30);
                return false;
            }
        }


        public Action backPivotBase() {
            return new BackPivotBase();
        }

        public class BackPivotTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backPivot.setPosition(0.05);
                return false;
            }
        }

        public Action backPivotTransfer() {
            return new BackPivotTransfer();
        }

        public class BackPivotInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                backPivot.setPosition(0);
                return false;
            }
        }

        public Action backPivotInit() {
            return new BackPivotTransfer();
        }


    }

    public static class ExtBack {

        private final Servo leftBackTransfer;
        private final Servo rightBackTransfer;
        private final Servo slideClaw;
        private final Servo SpecClaw;


        public ExtBack(HardwareMap hwMap) {
            leftBackTransfer = hwMap.get(Servo.class, "BTleft");
            rightBackTransfer = hwMap.get(Servo.class, "BTright");
            slideClaw = hwMap.get(Servo.class, "slide claw");
            SpecClaw = hwMap.get(Servo.class, "specimen claw");


            rightBackTransfer.setDirection(Servo.Direction.REVERSE);
        }

        public class specClawRel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpecClaw.setPosition(.5);
                return false;
            }
        }

        public Action specClawRel() {
            return new specClawRel();
        }

        public class specClawGo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SpecClaw.setPosition(0);
                return false;
            }
        }

        public Action specClawGo() {
            return new specClawGo();
        }





        public class SlidePivotBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftBackTransfer.setPosition(1);
                rightBackTransfer.setPosition(1);
                return false;
            }
        }

        public Action slidePivotBase() {
            return new SlidePivotBase();
        }

        public class SlidePivotDrop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftBackTransfer.setPosition(.35);
                rightBackTransfer.setPosition(.35);
                return false;
            }
        }
        public Action slidePivotDrop() {
            return new SlidePivotDrop();
        }

        public class SlideClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(.65);
                return false;
            }
        }

        public Action slideClawOpen() {
            return new SlideClawOpen();
        }

        public class SlideClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(0.41);
                return false;
            }
        }

        public Action slideClawClose() {
            return new SlideClawClose();
        }

    }

    @Override
    public void runOpMode() {


        Pose2d initPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);
        Slides slides = new Slides(hardwareMap);
        ExtFront extFront = new ExtFront(hardwareMap);
        ExtBack extBack = new ExtBack(hardwareMap);


        Actions.runBlocking(
                new ParallelAction(
                        extFront.clawOpen(),
                        extFront.transferIn(),
                        extFront.backPivotInit(),
                        extFront.frontPivotTransfer(),
                        extFront.wristInit(),
                        extBack.slidePivotBase(),
                        extBack.slideClawClose(),
                        slides.slidesDown()
                )
        );

        waitForStart();



        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .stopAndAdd(slides.slidesUp())
                        .strafeToLinearHeading(new Vector2d(-15,33), Math.toRadians(270))
                        .stopAndAdd(slides.slidesSpec())
                        .stopAndAdd(extBack.specClawRel())
                        .waitSeconds(.2)
                        .splineTo(new Vector2d(22,22), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(22,54), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(32,54), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(32,7), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(32,5), Math.toRadians(90))
                        .waitSeconds(.2)
                        .stopAndAdd(slides.slidesDown())
                        .waitSeconds(.15)
                        .strafeToLinearHeading(new Vector2d(32,0), Math.toRadians(90))
                        .stopAndAdd(extBack.specClawGo())
                        .waitSeconds(.1)
                        .stopAndAdd(slides.slidesUp())
                        .strafeToLinearHeading(new Vector2d(-13,33), Math.toRadians(270))
                        .stopAndAdd(slides.slidesSpec())
                        .stopAndAdd(extBack.specClawRel())
                        .waitSeconds(.1)
                        .strafeToLinearHeading(new Vector2d(32,3), Math.toRadians(90))
                        .waitSeconds(.1)
                        .stopAndAdd(slides.slidesDown())
                        .strafeToLinearHeading(new Vector2d(32,0), Math.toRadians(90))
                        .stopAndAdd(extBack.specClawGo())
                        .waitSeconds(.1)
                        .stopAndAdd(slides.slidesUp())
                        .strafeToLinearHeading(new Vector2d(-11,33), Math.toRadians(270))
                        .stopAndAdd(slides.slidesSpec())
                        .stopAndAdd(extBack.specClawRel())
                        .waitSeconds(.1)
                        .strafeToLinearHeading(new Vector2d(35,5), Math.toRadians(180))
                        .stopAndAdd(slides.slidesDown())



                       /* .strafeToLinearHeading(new Vector2d(32,54), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(45,54), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(45,5), Math.toRadians(270))

                        */



                        .waitSeconds(60)
                        .strafeToLinearHeading(new Vector2d(100,100), Math.toRadians(45))

                        .build()
        );


        if (isStopRequested()) return;


    }
}

//35 to the right (not a strafe tho) 12 to the left 20,22