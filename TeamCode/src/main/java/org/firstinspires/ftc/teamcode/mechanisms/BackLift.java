package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BackLift {
    private DcMotor slidesR;
    private DcMotor slidesL;
    private Servo slidePivot;
    private Servo slideClaw;
    private Servo specimenClaw;

    public void init(HardwareMap hwMap) {
        slidesR = hwMap.get(DcMotor.class, "slidesR");
        slidesL = hwMap.get(DcMotor.class, "slidesL");
        slidePivot = hwMap.get(Servo.class, "slide pivot");
        slideClaw = hwMap.get(Servo.class, "slide claw");
        specimenClaw = hwMap.get(Servo.class, "specimen claw");

        //Slides
        slidesR.setDirection(DcMotor.Direction.REVERSE);
        slidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivot.setDirection(Servo.Direction.REVERSE);

        slideClawOpen();
        slidePivotBase();
        specimenOpen();
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
        setSlides(0);
    }

    public void slidesTop() {
        setSlides(2250);
    }

    public void slidesMiddle() {
        setSlides(800);
    }

    public void slidesSpecimenGrab() {
        setSlides(0);
    }

    public void slidesSpecimenHang() {
        setSlides(950);
    }

    public void slidesSpecimenPreHang() {
        setSlides(1350);
    }

    public void slidesSpec(){setSlides(500);}

    /**
     * The following 2 (+1) loops all have to do with SlidePivot positions.
     */
    public void setSlidePivot(double pos) {
        slidePivot.setPosition(pos);
    }

    public void slidePivotBase() {
        setSlidePivot(0);
    }

    public void slidePivotDrop() {
        setSlidePivot(0.8);
    }

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

    /**
     * The following 2 (+1) are specimen claw positions.
     */
    public void setSpecimenClaw(double pos) {
        specimenClaw.setPosition(pos);
    }

    public void specimenOpen() {
        setSpecimenClaw(0.3);
    }

    public void specimenClose() {
        setSpecimenClaw(0.0);
    }

    public void climbTop(){setSlides(1800);}

    public void climbBottom(){setSlides(1200);}
}
