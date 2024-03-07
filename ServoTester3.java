package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Arm/Wrist Tester")
public class ServoTester3 extends OpMode {
    Servo armL;
    Servo armR;
    Servo wrist;
    boolean wristSelected = false;

    @Override
    public void init() {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armL.setDirection(Servo.Direction.FORWARD);
        armR.setDirection(Servo.Direction.REVERSE);
        armL.setPosition(0.4);
        armR.setPosition(0.4);
        wrist.setPosition(1);
    }

    @Override
    public void loop() {
        if (wristSelected) {
            wrist.setPosition(
                    wrist.getPosition() +
                            (gamepad1.right_trigger - gamepad1.left_trigger) / 1000
            );
        } else {
            double newArmPosition = armL.getPosition() +
                    (gamepad1.right_trigger - gamepad1.left_trigger) / 2000;
            armL.setPosition(newArmPosition);
            armR.setPosition(newArmPosition);
        }
        if (gamepad1.left_bumper) wristSelected = false;
        if (gamepad1.right_bumper) wristSelected = true;
        String lb1 = wristSelected ? "" : "[";
        String rb1 = wristSelected ? "" : "]";
        String lb2 = wristSelected ? "[" : "";
        String rb2 = wristSelected ? "]" : "";
        telemetry.addData(lb1 + "Arm" + rb1, armL.getPosition());
        telemetry.addData(lb2 + "Wrist" + rb2, wrist.getPosition());
    }
}
