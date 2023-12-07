/**
 * Controls the end-effector based on input from a gamepad.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Effector {
    private Servo armRotatorLeft;
    private Servo armRotatorRight;
    private Servo handActuator;
    private Servo pincerLeft;
    private Servo pincerRight;
    private Servo wristRotator;

    // Todo: Fix these fake values
    final double ACTUATOR_CLOSED_POSITION = 0;
    final double ACTUATOR_GRIP_POSITION = 0.1;
    final double ARM_FRONT_LIMIT = 0.1;
    final int LIFT_DOWN_POSITION = 200;
    final double ARM_FRONT_LIMIT_WHEN_LIFT_DOWN = 0.4;
    final double INPUT_SCALE_FACTOR = 0.1;

    public void init(Servo armRotatorLeft, Servo armRotatorRight, Servo wristRotator, Servo handActuator, Servo pincerLeft, Servo pincerRight) {
        this.armRotatorLeft = armRotatorLeft;
        this.armRotatorRight = armRotatorRight;
        this.wristRotator = wristRotator;
        this.handActuator = handActuator;
        this.pincerLeft = pincerLeft;
        this.pincerRight = pincerRight;

        armRotatorLeft.setDirection(Servo.Direction.FORWARD);
        armRotatorRight.setDirection(Servo.Direction.REVERSE);
        wristRotator.setDirection(Servo.Direction.REVERSE);
        handActuator.setDirection(Servo.Direction.REVERSE);
        pincerLeft.setDirection(Servo.Direction.FORWARD);
        pincerRight.setDirection(Servo.Direction.REVERSE);

        pincerLeft.scaleRange(ACTUATOR_CLOSED_POSITION, ACTUATOR_GRIP_POSITION);
        pincerRight.scaleRange(ACTUATOR_CLOSED_POSITION, ACTUATOR_GRIP_POSITION);
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

        handActuator.setPosition(0.5 * (gamepad.left_stick_x + 1));

        if (gamepad.left_bumper) {
            pincerLeft.setPosition(1);
        } else {
            pincerLeft.setPosition(0);
        }
        if (gamepad.right_bumper) {
            pincerRight.setPosition(1);
        } else {
            pincerRight.setPosition(0);
        }
    }
}
