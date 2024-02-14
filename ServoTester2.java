package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Arm/Hand Tester")
public class ServoTester2 extends OpMode {
    Servo armL;
    Servo armR;
    Servo hand;
    boolean handSelected = false;

    @Override
    public void init() {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        hand = hardwareMap.get(Servo.class, "hand");
        armL.setDirection(Servo.Direction.FORWARD);
        armR.setDirection(Servo.Direction.REVERSE);
        armL.setPosition(0.4);
        armR.setPosition(0.4);
        hand.setPosition(0.72);
    }

    @Override
    public void loop() {
        if (handSelected) {
            hand.setPosition(
                    hand.getPosition() +
                            (gamepad1.right_trigger - gamepad1.left_trigger) / 1000
            );
        } else {
            double newArmPosition = armL.getPosition() +
                    (gamepad1.right_trigger - gamepad1.left_trigger) / 2000;
            armL.setPosition(newArmPosition);
            armR.setPosition(newArmPosition);
        }
        if (gamepad1.left_bumper) handSelected = false;
        if (gamepad1.right_bumper) handSelected = true;
        String lb1 = handSelected ? "" : "[";
        String rb1 = handSelected ? "" : "]";
        String lb2 = handSelected ? "[" : "";
        String rb2 = handSelected ? "]" : "";
        telemetry.addData(lb1 + "Arm" + rb1, armL.getPosition());
        telemetry.addData(lb2 + "Hand" + rb2, hand.getPosition());
    }
}
