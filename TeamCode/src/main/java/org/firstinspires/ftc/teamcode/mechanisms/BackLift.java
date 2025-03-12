package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BackLift {
    private DcMotor slidesR;
    private DcMotor slidesL;
    private Servo slideClaw;
    private Servo leftBackTransfer;
    private Servo rightBackTransfer;
    private Servo specimenClaw;


    public void init(HardwareMap hwMap) {
        slidesR = hwMap.get(DcMotor.class, "slidesR");
        slidesL = hwMap.get(DcMotor.class, "slidesL");
        slideClaw = hwMap.get(Servo.class, "slide claw");
        leftBackTransfer = hwMap.get(Servo.class, "BTleft");
        rightBackTransfer = hwMap.get(Servo.class, "BTright");
        specimenClaw = hwMap.get(Servo.class, "specimen claw");

        //Slides
        slidesR.setDirection(DcMotor.Direction.REVERSE);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackTransfer.setDirection(Servo.Direction.REVERSE);

        slideClawOpen();
        transfergrab();
        specimenClose();
        initslides();
    }

    /**
     * The following 6 (+1) loops all have to do with different slide position. I don't know what the one with setSlides(1350) does.
     *
     */
    public void setSlides(int slidePos) {
        slidesL.setTargetPosition(slidePos);
        slidesR.setTargetPosition(slidePos);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesL.setPower(1);
        slidesR.setPower(1);
    }

    public void slidesBase() {
        setSlides(20);
    }

    public void slidesTop() {
        setSlides(2700);
    }

    public void slidesMiddle() {
        setSlides(800);
    }


    public void slidesSpecimenHang() {
        setSlides(1400);
    }

    public void slidesSpecimenPreHang() {
        setSlides(1850);
    }

    public void specslides(){
        setSlides(300);
    }

    public void initslides(){
        setSlides(0);
    }

    /**
     * The following 2 (+1) loops all have to do with SlidePivot positions.
     */

    /**
     * The following 2 (+1) are slide claw positions.
     */

    public void setSlideClaw(double pos) {
        slideClaw.setPosition(pos);
    }

    public void slideClawOpen() {
        setSlideClaw(.65);
    }

    public void slideClawClose() {
        setSlideClaw(0.41);
    }

    public void setTransfer(double L, double R) {
        leftBackTransfer.setPosition(L);
        rightBackTransfer.setPosition(R);
    }

    public void transfergrab() {
        setTransfer(1, 1);
    }
public  void transferdrop(){
        setTransfer(.35 ,.35);
    }

    /**
     *

     * The following 2 (+1) are specimen claw positions.
     */

    public void climbTop(){setSlides(2500);}

    public void climbBottom(){setSlides(1500);}

    public void setSpecimenClaw(double pos) {
        specimenClaw.setPosition(pos);
    }

    public void specimenOpen() {
        setSpecimenClaw(.5 );
    }

    public void specimenClose() {
        setSpecimenClaw(0);
    }

    public void resetSlidesEncoders() {
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
