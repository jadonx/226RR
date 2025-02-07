package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@Config
@TeleOp(name = "Tester - Wrist")
public class WristTester extends OpMode {
    public Servo claw, wrist, rotate;
    public ElapsedTime runtime;


    public static double  wristPos, rotatePos;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){
        claw = hardwareMap.get(Servo.class, "clawServo");
        rotate = hardwareMap.get(Servo.class, "rotateServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");


        claw.setPosition(0.5);
        rotate.setPosition(0.5);
        wrist.setPosition(0.5);

    }

    @Override
    public void loop(){

        if (gamepad1.a) {
            wrist.setPosition(wristPos);
        }
        if (gamepad1.x) {
            rotate.setPosition(rotatePos);
        }

        if(gamepad1.right_bumper) {
            claw.setPosition(0);
        }
        if(gamepad1.left_bumper) {
            claw.setPosition(0.5);
        }

        telemetry.addData("wristPos ", wristPos);
        telemetry.addData("rotatePos ", rotatePos);
        telemetry.update();
    }
}