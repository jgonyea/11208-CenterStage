package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GamePadTester")
public class KardiaGamePadTester extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        // B, A, X, Y buttons
        telemetry.addData("B: ", gamepad1.b);
        telemetry.addData("A: ", gamepad1.a);
        telemetry.addData("X: ", gamepad1.x);
        telemetry.addData("Y: ", gamepad1.y);

        // D-pads
        telemetry.addData("d-Up: ", gamepad1.dpad_up);
        telemetry.addData("d_Down: ", gamepad1.dpad_down);
        telemetry.addData("d-Right: ", gamepad1.dpad_right);
        telemetry.addData("d-Left: ", gamepad1.dpad_left);

        // Left/Right bumpers
        telemetry.addData("LB: ", gamepad1.left_bumper);
        telemetry.addData("RB: ", gamepad1.right_bumper);

        // Triggers
        telemetry.addData("R-Trigger: ", gamepad1.right_trigger);
        telemetry.addData("L-Trigger: ", gamepad1.left_trigger);

        // Left Joystick x/y
        telemetry.addData("Left-jStick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left-jStick Y: ", gamepad1.left_stick_y);

        // Right Joystick x/y
        telemetry.addData("Right-jStick X: ", gamepad1.right_stick_x);
        telemetry.addData("Right-jStick Y: ", gamepad1.right_stick_y);

        // Start/Back buttons
        telemetry.addData("Start: ", gamepad1.start);
        telemetry.addData("Back: ", gamepad1.back);
    }
}

