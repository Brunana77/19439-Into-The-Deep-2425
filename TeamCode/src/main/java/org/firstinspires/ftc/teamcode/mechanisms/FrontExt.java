package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontExt {
    private Servo backPivot;
    private Servo frontPivot;
    private Servo wristClaw;
    private Servo frontClaw;
    private Servo leftTransfer;
    private Servo rightTransfer;

    public void init(HardwareMap hwMap) {
        wristClaw = hwMap.get(Servo.class, "wrist");
        backPivot = hwMap.get(Servo.class, "backpivot");
        frontPivot = hwMap.get(Servo.class, "frontpivot");
        frontClaw = hwMap.get(Servo.class, "frontclaw");
        leftTransfer = hwMap.get(Servo.class, "left");
        rightTransfer = hwMap.get(Servo.class, "right");

        //front claw system
        backPivot.setDirection(Servo.Direction.REVERSE);
        frontPivot.setDirection(Servo.Direction.REVERSE);
        frontClaw.setDirection(Servo.Direction.REVERSE);
        rightTransfer.setDirection(Servo.Direction.REVERSE);

        backPivotBase();
        frontPivotTransfer();
        wristInit();
        frontClawOpen();
        transferIn();
    }

    /**
    * The following 4 (+1) void loops are all for transfers. On foenem grave.
     */
    public void setTransfer(double L, double R) {
        leftTransfer.setPosition(L);
        rightTransfer.setPosition(R);
    }

    public void transferIn() {
        setTransfer(0.69, 0.69);
    }

    public void transferFullIn() {
        setTransfer(0.65,0.65);
    }

    public void transferMiddle() {
        setTransfer(0.85, 0.85);
    }
    public void transferExtend() {
        setTransfer(1.0, 1);
    }

    /**
     * The following 2 (+1) are claw positions.
     */

    public void setFrontClaw(double pos) {
        frontClaw.setPosition(pos);
    }

    public void frontClawOpen() {
        setFrontClaw(0.65);
    }

    public void frontClawGrab() {
        setFrontClaw(0);
    }

    /**
     * The following 2 (+1) are wrist loops.
     *
     */
    public void setWristClaw(double pos) {
        wristClaw.setPosition(pos);
    }

    public void wristInit() {
        setWristClaw(0.43);
    }

    public void wristMiddle(){setWristClaw(0.615);}

    public void wristleftMiddle(){setWristClaw(0.245);}
    public void wristRotate() {
        setWristClaw(0.8);
    }

    /**
     * The following 3 (+1) positions are for the front pivot servo.
     *
     */
    public void setFrontPivot(double pos) {
        frontPivot.setPosition(pos);
    }

    public void frontPivotBase() {
        setFrontPivot(0.7);
    }

    public void frontPivotGrab() {
        setFrontPivot(.95);
    }

    public void frontPivotTransfer() {
        setFrontPivot(0.2);
    }

    /**
     * The following 4 (+1) positions are for the back pivot servo.
     *
     */
    public void setBackPivot(double pos) {
        backPivot.setPosition(pos);
    }

    public void backPivotBase () {
        setBackPivot(0.32);
    }

    public void backPivotTransfer() {
        setBackPivot(0.0);
    }

public void backPivotcimb() {setBackPivot(.5);}

}