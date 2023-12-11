/**
 * Controls the end-effector based on input from an autonomous OpMode.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoEffector {
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
    private final static double STAGED_LIFT_TIME = 3.0;

    private EffectorState currentState;
    private boolean movingToDrivingState;


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
        movingToDrivingState = false;
    }

    public void setPincers(boolean leftGripping, boolean rightGripping) {
        if (currentState == EffectorState.INTAKE || currentState == EffectorState.SCORING) {
            if (leftGripping) {
                pincerLeft.setPosition(PINCER_GRIPPING_POSITION);
            } else {
                pincerLeft.setPosition(PINCER_CLOSED_POSITION);
            }
            if (rightGripping) {
                pincerRight.setPosition(PINCER_GRIPPING_POSITION);
            } else {
                pincerRight.setPosition(PINCER_CLOSED_POSITION);
            }
        } else {
            throw new IllegalStateException("Cannot operate pincers in " + currentState);
        }
    }

    public void moveToIntake() {
        if (currentState == EffectorState.DRIVING) {
            movingToDrivingState = false;
            currentState = EffectorState.STAGED_INTAKE;
            timer.reset();
        } else {
            throw new IllegalStateException("Cannot move to intake from " + currentState);
        }
    }

    public void moveToScoring() {
        if (currentState == EffectorState.DRIVING) {
            movingToDrivingState = false;
            currentState = EffectorState.STAGED_LIFT;
            timer.reset();
        } else {
            throw new IllegalStateException("Cannot move to scoring from " + currentState);
        }
    }

    public void moveToDriving() {
        if (currentState == EffectorState.INTAKE) {
            movingToDrivingState = true;
            currentState = EffectorState.STAGED_INTAKE;
            timer.reset();
        } else if (currentState == EffectorState.SCORING) {
            movingToDrivingState = true;
            currentState = EffectorState.STAGED_LIFT;
            timer.reset();
        } else {
            throw new IllegalStateException("Cannot move to driving from " + currentState);
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

                if (timer.seconds() > STAGED_INTAKE_TIME) {
                    if (movingToDrivingState){
                        currentState = EffectorState.DRIVING;
                    } else {
                        currentState = EffectorState.INTAKE;
                    }
                }
                break;

            case INTAKE:
                armRotatorLeft.setPosition(ARM_INTAKE_POSITION);
                armRotatorRight.setPosition(ARM_INTAKE_POSITION);
                handActuator.setPosition(HAND_INTAKE_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);
                break;

            case STAGED_LIFT:
                armRotatorRight.setPosition(ARM_STAGED_LIFT_POSITION);
                wristRotator.setPosition(WRIST_INTAKE_POSITION);

                if (timer.seconds() > STAGED_LIFT_TIME) {
                    if (movingToDrivingState) {
                        currentState = EffectorState.DRIVING;
                    } else {
                        currentState = EffectorState.SCORING;
                    }
                }
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

    public String getCurrentState(){
        return currentState.name();
    }
}
