package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="servo test thing")
public class ServoTester extends OpMode {
    Servo leftPincer;
    Servo rightPincer;
    Servo currentPincer;

    @Override
    public void init() {
        leftPincer = hardwareMap.get(Servo.class, "pincerL");
        rightPincer = hardwareMap.get(Servo.class, "pincerR");
        currentPincer = leftPincer;
        leftPincer.setPosition(0.4422);
        rightPincer.setPosition(0.4939);
    }

    @Override
    public void loop() {
        currentPincer.setPosition(
                currentPincer.getPosition() + (
                        gamepad1.right_trigger - gamepad1.left_trigger
                        ) / 1000
        );
        telemetry.addData("Current Pincer Position", currentPincer.getPosition());
        if (gamepad1.left_bumper)
            currentPincer = leftPincer;
        if (gamepad1.right_bumper)
            currentPincer = rightPincer;
    }
}
