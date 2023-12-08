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
    private enum EffectorState {
        NORMAL_POSE,
        MOVING_TO_INTAKE,
        INTAKE_POSE,
        MOVING_TO_NORMAL,
        MOVING_BACKWARD_TURN_RESTRICTED,
        MOVING_BACKWARD_AND_TURNING,
        PLACING_POSE,
        MOVING_FORWARD_AND_TURNING,
        MOVING_FORWARD_TURN_RESTRICTED
    }

    // Todo: Fix these fake values
    private final static double ARM_NORMAL_POSITION = 0.2;
    private final static double WRIST_NORMAL_POSITION = 0;
    private final static double HAND_NORMAL_POSITION = 0.3;
    private final static long NORMAL_TO_INTAKE_TIME = 500_000_000;
    private final static double ARM_INTAKE_POSITION = 0.1;
    private final static double WRIST_INTAKE_POSITION = -0.1;
    private final static long INTAKE_TO_NORMAL_TIME = 500_000_000;
    private final static long NORMAL_TO_TURNING_TIME = 1_000_000_000;
    private final static double ARM_POSITION_TO_BEGIN_TURNING = 0.5;
    private final static long TURNING_TO_PLACING_TIME = 1_000_000_000;
    private final static double HAND_PLACING_POSITION = 0.7;
    private final static double ARM_PLACING_POSITION = 0.8;
    private final static double WRIST_PLACING_POSITION = 0.2;
    private final static double PINCER_CLOSED_POSITION = 0;
    private final static double PINCER_GRIPPING_POSITION = 0.2;
    private final static long PLACING_TO_TURNING_TIME = 1_000_000_000;
    private final static long TURNING_TO_NORMAL_TIME = 1_000_000_000;

    private EffectorState currentState;
    private long lastStateChangeTime;
    private boolean aPressed;
    private boolean xPressed;
    private boolean bPressed;

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

        // Todo: move servos to normal positions

        currentState = EffectorState.NORMAL_POSE;
        aPressed = false;
        xPressed = false;
        bPressed = false;
    }

    public void moveEffector(Gamepad gamepad) {
        // Detect button presses only once
        boolean a = false;
        if (gamepad.a && !aPressed) {
            aPressed = true;
            a = true;
        }
        if (!gamepad.a) {
            aPressed = false;
        }

        boolean x = false;
        if (gamepad.x && !xPressed) {
            xPressed = true;
            x = true;
        }
        if (!gamepad.x) {
            xPressed = false;
        }

        boolean b = false;
        if (gamepad.b && !bPressed) {
            bPressed = true;
            b = true;
        }
        if (!gamepad.b) {
            bPressed = false;
        }

        // Move servos
        switch (currentState) {
            case NORMAL_POSE:
                if (b) {
                    armRotatorLeft.setPosition(ARM_INTAKE_POSITION);
                    armRotatorRight.setPosition(ARM_INTAKE_POSITION);
                    wristRotator.setPosition(WRIST_INTAKE_POSITION);
                    currentState = EffectorState.MOVING_TO_INTAKE;
                    lastStateChangeTime = System.nanoTime();
                }
                if (x) {
                    armRotatorLeft.setPosition(ARM_POSITION_TO_BEGIN_TURNING);
                    armRotatorRight.setPosition(ARM_POSITION_TO_BEGIN_TURNING);
                    currentState = EffectorState.MOVING_BACKWARD_TURN_RESTRICTED;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case MOVING_TO_INTAKE:
                if (System.nanoTime() - lastStateChangeTime >= NORMAL_TO_INTAKE_TIME) {
                    currentState = EffectorState.INTAKE_POSE;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case INTAKE_POSE:
                if (gamepad.left_bumper) {
                    pincerLeft.setPosition(PINCER_GRIPPING_POSITION);
                }
                if (gamepad.right_bumper) {
                    pincerRight.setPosition(PINCER_GRIPPING_POSITION);
                }
                if (a) {
                    armRotatorLeft.setPosition(ARM_NORMAL_POSITION);
                    armRotatorRight.setPosition(ARM_NORMAL_POSITION);
                    wristRotator.setPosition(WRIST_NORMAL_POSITION);
                    currentState = EffectorState.MOVING_TO_NORMAL;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case MOVING_TO_NORMAL:
                if (System.nanoTime() - lastStateChangeTime >= INTAKE_TO_NORMAL_TIME) {
                    currentState = EffectorState.NORMAL_POSE;
                    lastStateChangeTime = System.nanoTime();
                }
            case MOVING_BACKWARD_TURN_RESTRICTED:
                if (System.nanoTime() - lastStateChangeTime >= NORMAL_TO_TURNING_TIME) {
                    armRotatorLeft.setPosition(ARM_PLACING_POSITION);
                    armRotatorRight.setPosition(ARM_PLACING_POSITION);
                    handActuator.setPosition(HAND_PLACING_POSITION);
                    wristRotator.setPosition(WRIST_PLACING_POSITION);
                    currentState = EffectorState.MOVING_BACKWARD_AND_TURNING;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case MOVING_BACKWARD_AND_TURNING:
                if (System.nanoTime() - lastStateChangeTime >= TURNING_TO_PLACING_TIME) {
                    currentState = EffectorState.PLACING_POSE;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case PLACING_POSE:
                if (gamepad.left_bumper) {
                    pincerLeft.setPosition(PINCER_CLOSED_POSITION);
                }
                if (gamepad.right_bumper) {
                    pincerRight.setPosition(PINCER_CLOSED_POSITION);
                }
                if (a) {
                    armRotatorLeft.setPosition(ARM_POSITION_TO_BEGIN_TURNING);
                    armRotatorRight.setPosition(ARM_POSITION_TO_BEGIN_TURNING);
                    handActuator.setPosition(HAND_NORMAL_POSITION);
                    wristRotator.setPosition(WRIST_NORMAL_POSITION);
                    currentState = EffectorState.MOVING_FORWARD_AND_TURNING;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case MOVING_FORWARD_AND_TURNING:
                if (System.nanoTime() - lastStateChangeTime >= PLACING_TO_TURNING_TIME) {
                    armRotatorLeft.setPosition(ARM_NORMAL_POSITION);
                    armRotatorRight.setPosition(ARM_NORMAL_POSITION);
                    currentState = EffectorState.MOVING_FORWARD_TURN_RESTRICTED;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
            case MOVING_FORWARD_TURN_RESTRICTED:
                if (System.nanoTime() - lastStateChangeTime >= TURNING_TO_NORMAL_TIME) {
                    currentState = EffectorState.NORMAL_POSE;
                    lastStateChangeTime = System.nanoTime();
                }
                break;
        }
    }
}
