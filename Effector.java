/**
 * Controls the end-effector based on input from a gamepad.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Effector {
    private Servo armRotatorLeft;
    private Servo armRotatorRight;
    private Servo handActuator;
    private Servo pincerLeft;
    private Servo pincerRight;
    private Servo wristRotator;
    private enum EffectorState {
        DRIVING,
        STAGED_INTAKE,
        INTAKE,
        STAGED_LIFT,
        SCORING,
    }

    // Todo: Fix these fake values
    private final static double ARM_DRIVING_POSITION = 0;
    private final static double ARM_INTAKE_POSITION = 0.1;
    private final static double ARM_SCORING_POSITION = 1;
    private final static double ARM_STAGED_INTAKE_POSITION = 0.2;
    private final static double ARM_STAGED_LIFT_POSITION = 0.5;

    private final static double HAND_DRIVING_POSITION = 0.2;
    private final static double HAND_INTAKE_POSITION = .8;
    private final static double HAND_SCORING_POSITION = 0;

    private final static double PINCER_CLOSED_POSITION = 0;
    private final static double PINCER_GRIPPING_POSITION = 0.4;

    private final static double WRIST_INTAKE_POSITION = 0;
    private final static double WRIST_SCORING_POSITION = 1;

    // Timings
    private ElapsedTime timer = new ElapsedTime();
    private final static double STAGED_INTAKE_TIME = 3.0;

    private EffectorState currentState;
    private boolean is_a_pressed = false;
    private boolean is_y_pressed = false;


    // Configure effector Servos.
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

        pincerLeft.setPosition(PINCER_GRIPPING_POSITION);
        pincerRight.setPosition(PINCER_GRIPPING_POSITION);
        wristRotator.setPosition(WRIST_INTAKE_POSITION);

        currentState = EffectorState.DRIVING;
    }

    public void moveEffector(Gamepad gp) {
        switch (currentState) {
            case DRIVING:
                armRotatorLeft.setPosition(ARM_DRIVING_POSITION);
                armRotatorRight.setPosition(ARM_DRIVING_POSITION);
                handActuator.setPosition(HAND_DRIVING_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);

                // Monitor for arm rotation buttons A & Y.
                if (gp.a && !gp.y && !is_a_pressed){
                    currentState = EffectorState.STAGED_INTAKE;
                    timer.reset();
                    this.is_a_pressed = true;
                    this.is_y_pressed = false;
                }
                if (gp.y && ! gp.a) {
                    currentState = EffectorState.STAGED_LIFT;
                    this.is_a_pressed = false;
                    this.is_y_pressed = true;
                }
                if (!gp.a) {
                    this.is_a_pressed = false;
                }
                break;

            case STAGED_INTAKE:
                armRotatorLeft.setPosition(ARM_STAGED_INTAKE_POSITION);
                armRotatorRight.setPosition(ARM_STAGED_INTAKE_POSITION);
                handActuator.setPosition(HAND_INTAKE_POSITION);

                if (timer.seconds() > STAGED_INTAKE_TIME) {
                    if (is_a_pressed){
                        currentState = EffectorState.INTAKE;
                    } else {
                        currentState = EffectorState.DRIVING;
                    }

                }
                break;

            case INTAKE:
                if (!gp.a) {
                    this.is_a_pressed = false;
                    currentState = EffectorState.STAGED_INTAKE;
                    break;
                }
                armRotatorLeft.setPosition(ARM_INTAKE_POSITION);
                armRotatorRight.setPosition(ARM_INTAKE_POSITION);
                handActuator.setPosition(HAND_INTAKE_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);

                movePincers(gp, pincerLeft, pincerRight);

                break;

            case STAGED_LIFT:
                armRotatorRight.setPosition(ARM_STAGED_LIFT_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);

                // Wait for 2nd press to go down.
                if (gp.a && !this.is_a_pressed) {
                    this.is_a_pressed = true;
                    currentState = EffectorState.DRIVING;
                }
                if (!gp.a) {
                    this.is_a_pressed = false;
                }

                // Wait for 2nd press to go up.
                if (gp.y && !this.is_y_pressed) {
                    this.is_y_pressed = true;
                    currentState = EffectorState.SCORING;
                }
                if (!gp.y) {
                    this.is_y_pressed = false;
                }
                break;

            case SCORING:
                armRotatorRight.setPosition(ARM_SCORING_POSITION);
                armRotatorLeft.setPosition(ARM_SCORING_POSITION);
                handActuator.setPosition(HAND_SCORING_POSITION);
                wristRotator.setPosition(WRIST_SCORING_POSITION);

                movePincers(gp, pincerLeft, pincerRight);

                if (gp.a) {
                    currentState = EffectorState.STAGED_LIFT;
                    is_a_pressed = true;
                }
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }

    private void movePincers(Gamepad gp, Servo pincerLeft, Servo pincerRight) {
        if (gp.left_bumper) {
            pincerLeft.setPosition(PINCER_CLOSED_POSITION);
        } else {
            pincerLeft.setPosition(PINCER_GRIPPING_POSITION);
        }
        if (gp.right_bumper) {
            pincerRight.setPosition(PINCER_CLOSED_POSITION);
        } else {
            pincerRight.setPosition(PINCER_GRIPPING_POSITION);
        }
    }

    public String getCurrentState(){
        return currentState.name();
    }
}
