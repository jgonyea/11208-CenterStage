/**
 * Controls the positioning of end-effector and pincers.
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
    public enum EffectorState {
        SCORING,
        STAGED_LIFT,
        DRIVING,
        STAGED_INTAKE,
        INTAKE
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

    private final static double PINCERL_CLOSED_POSITION = 0.4422;
    private final static double PINCERR_CLOSED_POSITION = 0.4939;
    private final static double PINCER_GRIP_OFFSET = 0.06;

    private final static double WRIST_INTAKE_POSITION = 1;
    private final static double WRIST_SCORING_POSITION = 0.16;

    // Timings
    private final ElapsedTime timer = new ElapsedTime();
    public final static int STAGED_INTAKE_TIME = 300;
    public final static double STAGED_LIFT_TIME = 1.0;

    private EffectorState currentState = EffectorState.DRIVING;
    private EffectorState desiredState = EffectorState.DRIVING;
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
        pincerLeft.setDirection(Servo.Direction.FORWARD);
        pincerRight.setDirection(Servo.Direction.FORWARD);

        // Move pincers to grip/ open positions.
        pincerLeft.setPosition(PINCERL_CLOSED_POSITION + PINCER_GRIP_OFFSET);
        pincerRight.setPosition(PINCERR_CLOSED_POSITION - PINCER_GRIP_OFFSET);

        // Move effector to initialized state.
        moveEffector(EffectorState.DRIVING);
    }

    /**
     * Callable method to move effector.
     */
    public void run(){
        // Check if we need to move effector.
        if (desiredState != currentState){
            moveEffector(nextState());
        }
        // todo: Check if we need to move pincers

    }

    /**
     * Update current state using gamepad input.
     * @param gp Gamepad input
     */
    public void manualUpdate(Gamepad gp) {
        switch (currentState) {
            case DRIVING:
                // Monitor for arm rotation buttons A & Y.
                if (gp.a && !gp.y && !is_a_pressed){
                    setDesiredState(EffectorState.STAGED_INTAKE);
                    this.is_a_pressed = true;
                    this.is_y_pressed = false;
                    timer.reset();
                    break;
                }
                if (gp.y && !gp.a) {
                    setDesiredState(EffectorState.STAGED_LIFT);
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
                // todo: Maybe we don't want this to drop pixels?
                if (is_a_pressed) {
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                }
                if (timer.milliseconds() > STAGED_INTAKE_TIME) {
                    if (is_a_pressed){
                        setDesiredState(EffectorState.INTAKE);
                    } else {
                        setDesiredState(EffectorState.DRIVING);
                    }
                    break;
                }
                break;

            case INTAKE:
                movePincers(gp, pincerLeft, pincerRight);
                if (!gp.a) {
                    this.is_a_pressed = false;
                    setDesiredState(EffectorState.STAGED_INTAKE);
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
                        setDesiredState(EffectorState.DRIVING);
                        break;
                    }
                    if (gp.y && !this.is_y_pressed) {
                        this.is_y_pressed = true;
                        setDesiredState(EffectorState.SCORING);
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
                    setDesiredState(EffectorState.STAGED_LIFT);
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                    is_a_pressed = true;
                    timer.reset();
                }
                break;

            default:
                throw new IllegalStateException("Unexpected value: " + currentState);
        }

        // Move effector.
        run();
    }

    /**
     * Moves effector to new position.
     * @param newPosition New position.
     */
    private void moveEffector(EffectorState newPosition) {
        switch (newPosition) {
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
                throw new IllegalStateException("Unexpected value: " + newPosition.name());
        }

        setCurrentState(newPosition);
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

    public EffectorState getCurrentState(){
        return currentState;
    }

    private void setCurrentState(EffectorState newState){
        this.currentState = newState;
        timer.reset();
    }

    public EffectorState getDesiredState() {
        return this.desiredState;
    }

    public void setDesiredState(EffectorState desiredState) {
        this.desiredState = desiredState;
        if (desiredState == currentState){
            return;
        }
        moveEffector(nextState());
    }

    /**
     * Calculates the next state based on current and desired positions.
     * @return nextPosition Next position
     */
    private EffectorState nextState() {
        EffectorState nextPosition = null;
        switch (currentState){
            case SCORING:
                if (desiredState == EffectorState.DRIVING || desiredState == EffectorState.STAGED_LIFT){
                    nextPosition = EffectorState.STAGED_LIFT;
                }
                break;
            case STAGED_LIFT:
                if (desiredState == EffectorState.SCORING){
                    nextPosition = EffectorState.SCORING;
                } else if (desiredState == EffectorState.DRIVING){
                    nextPosition = EffectorState.DRIVING;
                }
                break;
            case DRIVING:
                if (desiredState == EffectorState.STAGED_LIFT || desiredState == EffectorState.SCORING){
                    nextPosition = EffectorState.STAGED_LIFT;
                } else if (desiredState == EffectorState.STAGED_INTAKE || desiredState == EffectorState.INTAKE){
                    nextPosition = EffectorState.STAGED_INTAKE;
                }
                break;
            case STAGED_INTAKE:
                if (desiredState == EffectorState.DRIVING){
                    nextPosition = EffectorState.DRIVING;
                } else if (desiredState == EffectorState.INTAKE){
                    nextPosition = EffectorState.INTAKE;
                }
                break;
            case INTAKE:
                if (desiredState == EffectorState.STAGED_INTAKE || desiredState == EffectorState.DRIVING){
                    nextPosition = EffectorState.STAGED_INTAKE;
                }
                break;
            default:
                nextPosition = EffectorState.DRIVING;
        }

        return nextPosition;
    }
}
