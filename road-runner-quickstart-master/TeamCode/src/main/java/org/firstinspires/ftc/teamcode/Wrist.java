package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Wrist {
    Servo wristServo;
    Servo rotateServo;
    OpMode opMode;

    public Wrist(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo"); //leftServo
        rotateServo = opMode.hardwareMap.get(Servo.class, "rotateServo"); //rightServo
    }

    public void Up() {
        wristServo.setPosition(0);
        rotateServo.setPosition(0.5);
    }

    public void Down() {
        wristServo.setPosition(1);
        rotateServo.setPosition(0.5);
    }

    public void ReadyPlaceSample() { // ____|
        wristServo.setPosition(0.35);
        rotateServo.setPosition(0.5);
    }

    public void PlaceSample() {//  ____\
        wristServo.setPosition(0); //0.45
        rotateServo.setPosition(0.5);
    }

    public void PickUp0() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.5);
    }

    public void PickUp45Right() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.35);
    }

    public void PickUp45Left() {
        wristServo.setPosition(1);
        rotateServo.setPosition(0.67);
    }

    public void PickUp90() {
        wristServo.setPosition(1);
        rotateServo.setPosition(0.5);
    }

    public void PickUpSpecimen() {}

    public boolean isUp() {
        if (wristServo.getPosition() != 0) {
            return false;
        }
        return true;
    }
}