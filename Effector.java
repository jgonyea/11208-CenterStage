/**
 * Controls the end-effector based on input from a gamepad.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Effector {
    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handRotator;
    Servo leftActuator;
    Servo rightActuator;
    Servo wristRotator;

    // Todo: Fix these fake values
    final double ACTUATOR_CLOSED_POSITION = 0;
    final double ACTUATOR_GRIP_POSITION = 0.1;
    final double ARM_FRONT_LIMIT = 0.1;
    final int LIFT_DOWN_POSITION = 200;
    final double ARM_FRONT_LIMIT_WHEN_LIFT_DOWN = 0.4;
    final double INPUT_SCALE_FACTOR = 0.1;

    public void init(Servo armRotatorLeft, Servo armRotatorRight, Servo wristRotator, Servo handRotator, Servo leftActuator, Servo rightActuator) {
        this.armRotatorLeft = armRotatorLeft;
        this.armRotatorRight = armRotatorRight;
        this.wristRotator = wristRotator;
        this.handRotator = handRotator;
        this.leftActuator = leftActuator;
        this.rightActuator = rightActuator;

        armRotatorLeft.setDirection(Servo.Direction.FORWARD);
        armRotatorRight.setDirection(Servo.Direction.REVERSE);
        wristRotator.setDirection(Servo.Direction.REVERSE);
        handRotator.setDirection(Servo.Direction.REVERSE);
        leftActuator.setDirection(Servo.Direction.FORWARD);
        rightActuator.setDirection(Servo.Direction.REVERSE);

        leftActuator.scaleRange(ACTUATOR_CLOSED_POSITION, ACTUATOR_GRIP_POSITION);
        rightActuator.scaleRange(ACTUATOR_CLOSED_POSITION, ACTUATOR_GRIP_POSITION);
    }

    public void moveEffector(Gamepad gamepad, int liftPosition) {
        double newTargetPosition = armRotatorLeft.getPosition() + (INPUT_SCALE_FACTOR * gamepad.left_stick_y);
        newTargetPosition = Math.max(ARM_FRONT_LIMIT, Math.min(1, newTargetPosition));
        if (liftPosition <= LIFT_DOWN_POSITION) {
            newTargetPosition = Math.max(ARM_FRONT_LIMIT_WHEN_LIFT_DOWN, newTargetPosition);
        }
        armRotatorLeft.setPosition(newTargetPosition);
        armRotatorRight.setPosition(newTargetPosition);

        wristRotator.setPosition(wristRotator.getPosition() + (INPUT_SCALE_FACTOR * gamepad.right_stick_y));

        handRotator.setPosition(0.5 * (gamepad.left_stick_x + 1));

        if (gamepad.left_bumper) {
            leftActuator.setPosition(1);
        } else {
            leftActuator.setPosition(0);
        }
        if (gamepad.right_bumper) {
            rightActuator.setPosition(1);
        } else {
            rightActuator.setPosition(0);
        }
    }
}
