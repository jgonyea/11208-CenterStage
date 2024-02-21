package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Front Pincer Tester")
public class FrontPincerTester extends OpMode {
    Servo leftServo;
    Servo rightServo;
    Servo current;

    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "frontpL");
        rightServo = hardwareMap.get(Servo.class, "frontpR");
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        current = leftServo;
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper)
            current = leftServo;
        if (gamepad1.right_bumper)
            current = rightServo;

        current.setPosition(
                current.getPosition() + (
                        gamepad1.right_trigger - gamepad1.left_trigger
                ) / 1500
        );
        telemetry.addData("Current Servo Position", current.getPosition());
    }
}
