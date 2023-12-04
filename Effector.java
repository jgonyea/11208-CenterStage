/**
 * Controls the end-effector based on input from gamepad2.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;


import com.qualcomm.robotcore.hardware.Servo;

public class Effector {
    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handRotator;
    Servo leftActuator;
    Servo rightActuator;

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
    }
}
