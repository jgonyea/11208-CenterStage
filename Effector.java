/**
 * Controls the end-effector based on input from gamepad2.
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

    // Todo: Fix these fake values
    final double CLOSED_POSITION = 0;
    final double GRIP_POSITION = 0.1;

    public void init(Servo armRotatorLeft, Servo armRotatorRight, Servo handRotator, Servo leftActuator, Servo rightActuator) {
        this.armRotatorLeft = armRotatorLeft;
        this.armRotatorRight = armRotatorRight;
        this.handRotator = handRotator;
        this.leftActuator = leftActuator;
        this.rightActuator = rightActuator;

        armRotatorLeft.setDirection(Servo.Direction.FORWARD);
        armRotatorRight.setDirection(Servo.Direction.REVERSE);
        handRotator.setDirection(Servo.Direction.REVERSE);
        leftActuator.setDirection(Servo.Direction.FORWARD);
        rightActuator.setDirection(Servo.Direction.REVERSE);

        leftActuator.scaleRange(CLOSED_POSITION, GRIP_POSITION);
        rightActuator.scaleRange(CLOSED_POSITION, GRIP_POSITION);
    }

    public void moveRobot(Gamepad gamepad2) {
        double newTargetPosition = armRotatorLeft.getPosition() + (gamepad2.left_stick_y / 10);
        newTargetPosition = Math.max(0, Math.min(1, newTargetPosition));
        armRotatorLeft.setPosition(newTargetPosition);
        armRotatorRight.setPosition(newTargetPosition);
    }
}
