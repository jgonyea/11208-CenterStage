/**
 * Controls the end-effector based on input from a gamepad.
 */
package org.firstinspires.ftc.teamcode;

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
        SCORING
    }

    // Values based on empirical testing.
    private final static double ARM_DRIVING_POSITION = 0.400;
    private final static double ARM_INTAKE_POSITION = 0.386;
    private final static double ARM_SCORING_POSITION = 0.890;
    private final static double ARM_STAGED_INTAKE_POSITION = 0.410;
    private final static double ARM_STAGED_LIFT_POSITION = 0.450;

    private final static double HAND_DRIVING_POSITION = 0.72;
    private final static double HAND_INTAKE_POSITION = 0.40;
    private final static double HAND_SCORING_POSITION = 0.258;

    private final static double PINCERL_CLOSED_POSITION = 0.43;
    private final static double PINCERR_CLOSED_POSITION = 0.52;
    private final static double PINCER_GRIP_OFFSET = 0.08;

    private final static double WRIST_INTAKE_POSITION = 1;
    private final static double WRIST_SCORING_POSITION = 0.16;

    // Timings
    private final ElapsedTime timer = new ElapsedTime();
    private final static double STAGED_INTAKE_TIME = 0.3;
    private final static double STAGED_LIFT_TIME = 1.0;

    private EffectorState currentState;
    private boolean is_a_pressed = false;
    private boolean is_y_pressed = false;
    private boolean movingTowardsDrivingState = false;

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
        pincerLeft.setDirection(Servo.Direction.FORWARD);
        pincerRight.setDirection(Servo.Direction.FORWARD);

        // Move pincers to grip/ open positions.
        pincerLeft.setPosition(PINCERL_CLOSED_POSITION + PINCER_GRIP_OFFSET);
        pincerRight.setPosition(PINCERR_CLOSED_POSITION - PINCER_GRIP_OFFSET);

        armRotatorLeft.setPosition(ARM_STAGED_LIFT_POSITION);
        armRotatorRight.setPosition(ARM_STAGED_LIFT_POSITION);
        handActuator.setPosition(HAND_DRIVING_POSITION);
        wristRotator.setPosition(WRIST_INTAKE_POSITION);

        currentState = EffectorState.STAGED_LIFT;
    }

    // Update current state using gamepad input.
    public void updateFromGamepad(Gamepad gp) {
        switch (currentState) {
            case DRIVING:
                // Monitor for arm rotation buttons A & Y.
                if (gp.a && !gp.y && !is_a_pressed){
                    currentState = EffectorState.STAGED_INTAKE;
                    this.is_a_pressed = true;
                    this.is_y_pressed = false;
                    timer.reset();
                    break;
                }
                if (gp.y && !gp.a) {
                    currentState = EffectorState.STAGED_LIFT;
                    this.is_a_pressed = false;
                    this.is_y_pressed = true;
                    timer.reset();
                    break;
                }
                if (!gp.a) {
                    this.is_a_pressed = false;
                }
                break;

            case STAGED_INTAKE:
                if (is_a_pressed) {
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                }
                if (timer.seconds() > STAGED_INTAKE_TIME) {
                    if (is_a_pressed){
                        currentState = EffectorState.INTAKE;
                    } else {
                        currentState = EffectorState.DRIVING;
                    }
                    break;
                }
                break;

            case INTAKE:
                movePincers(gp, pincerLeft, pincerRight);
                if (!gp.a) {
                    this.is_a_pressed = false;
                    currentState = EffectorState.STAGED_INTAKE;
                    timer.reset();
                    break;
                }
                break;

            case STAGED_LIFT:
                if (timer.seconds() > STAGED_LIFT_TIME){
                    wristRotator.setPosition(WRIST_INTAKE_POSITION);

                    // Wait for 2nd press to go down.
                    if (gp.a && !this.is_a_pressed) {
                        this.is_a_pressed = true;
                        currentState = EffectorState.DRIVING;
                        break;
                    }
                    if (gp.y && !this.is_y_pressed) {
                        this.is_y_pressed = true;
                        currentState = EffectorState.SCORING;
                        break;
                    }
                }

                // Wait for 1st press to go up.
                if (!gp.a) {
                    this.is_a_pressed = false;
                }
                if (!gp.y) {
                    this.is_y_pressed = false;
                }
                break;

            case SCORING:
                movePincers(gp, pincerLeft, pincerRight);
                if (gp.a) {
                    currentState = EffectorState.STAGED_LIFT;
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                    is_a_pressed = true;
                    timer.reset();
                }
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }

    // Update current state based on movingTowardsDrivingState.
    public void updateWithoutGamepad() {
        switch (currentState) {
            case STAGED_INTAKE:
                if (!movingTowardsDrivingState) {
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                }

                if (timer.seconds() > STAGED_INTAKE_TIME) {
                    if (movingTowardsDrivingState) {
                        currentState = EffectorState.DRIVING;
                    } else {
                        currentState = EffectorState.INTAKE;
                    }
                }

                break;

            case INTAKE:
                pincerLeft.setPosition(PINCERL_CLOSED_POSITION + PINCER_GRIP_OFFSET);
                pincerRight.setPosition(PINCERR_CLOSED_POSITION - PINCER_GRIP_OFFSET);

                break;

            case STAGED_LIFT:
                if (timer.seconds() > STAGED_LIFT_TIME) {
                    wristRotator.setPosition(WRIST_INTAKE_POSITION);

                    if (movingTowardsDrivingState) {
                        currentState = EffectorState.DRIVING;
                    } else {
                        currentState = EffectorState.SCORING;
                    }
                }
        }
    }

    public void moveEffector() {
        switch (currentState) {
            case DRIVING:
                armRotatorLeft.setPosition(ARM_DRIVING_POSITION);
                armRotatorRight.setPosition(ARM_DRIVING_POSITION);
                handActuator.setPosition(HAND_DRIVING_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);
                break;

            case STAGED_INTAKE:
                armRotatorLeft.setPosition(ARM_STAGED_INTAKE_POSITION);
                armRotatorRight.setPosition(ARM_STAGED_INTAKE_POSITION);
                handActuator.setPosition(HAND_INTAKE_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);
                break;

            case INTAKE:
                armRotatorLeft.setPosition(ARM_INTAKE_POSITION);
                armRotatorRight.setPosition(ARM_INTAKE_POSITION);
                handActuator.setPosition(HAND_INTAKE_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);
                break;

            case STAGED_LIFT:
                armRotatorLeft.setPosition(ARM_STAGED_LIFT_POSITION);
                armRotatorRight.setPosition(ARM_STAGED_LIFT_POSITION);
                handActuator.setPosition(HAND_DRIVING_POSITION);
                break;

            case SCORING:
                armRotatorRight.setPosition(ARM_SCORING_POSITION);
                armRotatorLeft.setPosition(ARM_SCORING_POSITION);
                handActuator.setPosition(HAND_SCORING_POSITION);
                wristRotator.setPosition(WRIST_SCORING_POSITION);
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }
    }

    private void movePincers(Gamepad gp, Servo pincerLeft, Servo pincerRight) {
        if (gp.left_bumper) {
            pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
        } else {
            pincerLeft.setPosition(PINCERL_CLOSED_POSITION + PINCER_GRIP_OFFSET);
        }
        if (gp.right_bumper) {
            pincerRight.setPosition(PINCERR_CLOSED_POSITION);
        } else {
            pincerRight.setPosition(PINCERR_CLOSED_POSITION - PINCER_GRIP_OFFSET);
        }
    }

    public String getCurrentState(){
        return currentState.name();
    }

    public void setCurrentState(EffectorState currentState, boolean movingTowardsDrivingState){
        this.currentState = currentState;
        this.movingTowardsDrivingState = movingTowardsDrivingState;
        timer.reset();
    }
}
