package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Pincer Tester")
public class ServoTester extends OpMode {
    Servo leftPincer;
    Servo rightPincer;
    Servo currentPincer;

    @Override
    public void init() {
        leftPincer = hardwareMap.get(Servo.class, "pincerL");
        rightPincer = hardwareMap.get(Servo.class, "pincerR");
        leftPincer.setPosition(0.4944);
        rightPincer.setPosition(0.4344);

        currentPincer = leftPincer;
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper)
            currentPincer = leftPincer;
        if (gamepad1.right_bumper)
            currentPincer = rightPincer;

        currentPincer.setPosition(
                currentPincer.getPosition() + (
                        gamepad1.right_trigger - gamepad1.left_trigger
                        ) / 1000
        );
        telemetry.addData("Current Pincer Position", currentPincer.getPosition());
    }
}
