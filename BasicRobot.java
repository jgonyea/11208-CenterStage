/**
 * Driver-controlled routine for basic drivetrain demo robot.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Drivetrain and spinner")
public class BasicRobot extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    DcMotor spin;
    boolean zeroed;
    boolean spinDir;
    double throttle;
    boolean throttlePressed;
    boolean invert;
    boolean invertPressed;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "RearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "RearRight");
        spin = hardwareMap.get(DcMotor.class, "Spinner");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        zeroed = true;
        spinDir = true;
        invert = false;
        invertPressed = false;
        throttle = 1;
        throttlePressed = false;
    }

    @Override
    public void loop() {
        // Invert button
        if (gamepad1.dpad_down && gamepad1.dpad_left && gamepad1.dpad_up && gamepad1.dpad_right && !invertPressed) {
            invertPressed = true;
            invert = !invert;
        }
        if (!(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right)) {
            invertPressed = false;
        }

        // Calculate values
        double x = -gamepad1.left_stick_x;
        double y = Math.max(-1, Math.min(1, gamepad1.left_stick_y + gamepad1.right_stick_y));
        if (invert) {
            x = -x;
            y = -y;
        }
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x,y);
        double turn = gamepad1.right_stick_x;

        // Calculate initial power results to motors.
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(
                Math.abs(sin),
                Math.abs(cos)
        );

        // Set power levels for each wheel.
        double powerFrontLeft = power * (cos / max) - turn;
        double powerFrontRight = power * (sin / max) + turn;
        double powerRearLeft = power * (sin / max) - turn;
        double powerRearRight = power * (cos / max) + turn;

        // Rescale power if beyond maximum.
        double scale = power + Math.abs(turn);
        if (scale > 1) {
            powerFrontLeft /= scale;
            powerFrontRight /= scale;
            powerRearLeft  /= scale;
            powerRearRight /= scale;
        }

        // Assign power to wheels.
        frontRight.setPower(powerFrontRight * throttle);
        frontLeft.setPower(powerFrontLeft * throttle);
        rearRight.setPower(powerRearRight * throttle);
        rearLeft.setPower(powerRearLeft * throttle);

        // Spin based on left trigger.
        if (gamepad1.left_trigger == 0 && !zeroed) {
            zeroed = true;
            spinDir = !spinDir;
        }
        if (gamepad1.left_trigger > 0) {
            zeroed = false;
        }
        if (spinDir) {
            spin.setPower(+gamepad1.left_trigger);
        } else {
            spin.setPower(-gamepad1.left_trigger);
        }

        // Throttle based on right trigger/button.
        if (gamepad1.right_bumper && !throttlePressed) {
            throttle = (1 - gamepad1.right_trigger);
        }
        if (!gamepad1.right_bumper && gamepad1.right_trigger == 0) {
            throttlePressed = false;
        }
    }
}
