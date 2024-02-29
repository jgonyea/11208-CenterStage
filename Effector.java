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
    private Servo frontPincerLeft;
    private Servo frontPincerRight;
    public enum EffectorState {
        SCORING,
        STAGED_LIFT,
        DRIVING,
        SWEEP_FRONT,
        STAGED_INTAKE,
        INTAKE
    }

    // Values based on empirical testing.
    private final static double ARM_DRIVING_POSITION = 0.400;
    private final static double ARM_INTAKE_POSITION = 0.3711;
    private final static double ARM_SCORING_POSITION = 0.890;
    private final static double ARM_STAGED_INTAKE_POSITION = 0.410;
    private final static double ARM_STAGED_LIFT_POSITION = 0.450;

    private final static double HAND_DRIVING_POSITION = 0.72;
    private final static double HAND_INTAKE_POSITION = 0.4;
    private final static double HAND_SCORING_POSITION = 0.258;

    private final static double PINCERL_CLOSED_POSITION = 0.5067;
    private final static double PINCERR_CLOSED_POSITION = 0.4461;
    private final static double PINCER_GRIP_OFFSET = 0.07;

    private final static double FRONT_PINCERL_CLOSED_POSITION = 0.1794;
    private final static double FRONT_PINCERR_CLOSED_POSITION = 0.8467;
    private final static double FRONT_PINCER_OPEN_OFFSET = 0.6033;
    public enum PincerState {
        GRIP,
        RELEASE
    }

    private final static double WRIST_INTAKE_POSITION = 1;
    private final static double WRIST_SCORING_POSITION = 0.16;

    // Timings
    private final ElapsedTime timer = new ElapsedTime();
    public final static long SWEEP_FRONT_TIME = 50;
    public final static long STAGED_INTAKE_TIME = 300;
    public final static long STAGED_LIFT_TIME = 500;

    private EffectorState currentState = EffectorState.DRIVING;
    private EffectorState desiredState = EffectorState.DRIVING;
    private PincerState tempLeftPincerPosition;
    private PincerState tempRightPincerPosition;
    private boolean is_a_pressed = false;
    private boolean is_b_pressed = false;
    private boolean is_y_pressed = false;

    // Configure effector Servos.
    public void init(Servo armRotatorLeft, Servo armRotatorRight, Servo wristRotator, Servo handActuator, Servo pincerLeft, Servo pincerRight, Servo frontPincerLeft, Servo frontPincerRight) {
        this.armRotatorLeft = armRotatorLeft;
        this.armRotatorRight = armRotatorRight;
        this.wristRotator = wristRotator;
        this.handActuator = handActuator;
        this.pincerLeft = pincerLeft;
        this.pincerRight = pincerRight;
        this.frontPincerLeft = frontPincerLeft;
        this.frontPincerRight = frontPincerRight;

        armRotatorLeft.setDirection(Servo.Direction.FORWARD);
        armRotatorRight.setDirection(Servo.Direction.REVERSE);
        pincerLeft.setDirection(Servo.Direction.FORWARD);
        pincerRight.setDirection(Servo.Direction.FORWARD);

        // Move effector to initialized state.
        moveEffector(EffectorState.DRIVING);

        // Move pincers to grip/ open positions.
        setPincerPosition(pincerLeft, PincerState.RELEASE);
        setPincerPosition(pincerRight, PincerState.RELEASE);
    }

    /**
     * Callable method to move effector.
     */
    public void run(){
        // Check if we need to move effector.
        if (desiredState != currentState){
            moveEffector(nextState());
        }
    }

    /**
     * Update current state using gamepad input.
     * @param gp Gamepad input
     */
    public void manualUpdate(Gamepad gp) {
        switch (currentState) {
            case DRIVING:
                // Monitor for arm rotation buttons A & Y.
                if (gp.a && !is_a_pressed){
                    tempLeftPincerPosition = PincerState.RELEASE;
                    tempRightPincerPosition = PincerState.RELEASE;
                    this.is_a_pressed = true;
                    this.is_y_pressed = false;
                    timer.reset();
                    setDesiredState(EffectorState.SWEEP_FRONT);
                    break;
                }
                if (gp.b && !is_b_pressed && (gp.left_bumper || gp.right_bumper)){
                    tempLeftPincerPosition = bumperToPincer(gp.left_bumper);
                    tempRightPincerPosition = bumperToPincer(gp.right_bumper);
                    this.is_b_pressed = true;
                    this.is_y_pressed = false;
                    timer.reset();
                    setDesiredState(EffectorState.SWEEP_FRONT);
                    break;
                }
                if (gp.y && !gp.a && !gp.b) {
                    this.is_a_pressed = false;
                    this.is_y_pressed = true;
                    timer.reset();
                    setDesiredState(EffectorState.STAGED_LIFT);
                    break;
                }
                if (!gp.a) {
                    this.is_a_pressed = false;
                }
                if (!gp.b) {
                    this.is_b_pressed = false;
                }
                break;

            case SWEEP_FRONT:
                if (timer.milliseconds() > SWEEP_FRONT_TIME) {
                    timer.reset();
                    setDesiredState(EffectorState.STAGED_INTAKE);
                }
                break;

            case STAGED_INTAKE:
                if (is_a_pressed || is_b_pressed) {
                    setPincerPosition(pincerLeft, tempLeftPincerPosition);
                    setPincerPosition(pincerRight, tempRightPincerPosition);
                }
                if (timer.milliseconds() > STAGED_INTAKE_TIME) {
                    if (is_a_pressed || is_b_pressed){
                        setDesiredState(EffectorState.INTAKE);
                    } else {
                        setDesiredState(EffectorState.DRIVING);
                    }
                    break;
                }
                break;

            case INTAKE:
                setPincerPosition(pincerLeft, PincerState.GRIP);
                setPincerPosition(pincerRight, PincerState.GRIP);
                if ((!gp.a && this.is_a_pressed) || (!gp.b && this.is_b_pressed)) {
                    this.is_a_pressed = false;
                    this.is_b_pressed = false;
                    setPincerPosition(frontPincerLeft, PincerState.RELEASE);
                    setPincerPosition(frontPincerRight, PincerState.RELEASE);
                    timer.reset();
                    setDesiredState(EffectorState.STAGED_INTAKE);
                    break;
                }
                break;

            case STAGED_LIFT:
                if (timer.milliseconds() > STAGED_LIFT_TIME){
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
                setPincerPosition(pincerLeft, bumperToPincer(gp.left_bumper));
                setPincerPosition(pincerRight, bumperToPincer(gp.right_bumper));
                if (gp.a) {
                    pincerLeft.setPosition(PINCERL_CLOSED_POSITION);
                    pincerRight.setPosition(PINCERR_CLOSED_POSITION);
                    is_a_pressed = true;
                    timer.reset();
                    setDesiredState(EffectorState.STAGED_LIFT);
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
                setPincerPosition(frontPincerLeft, PincerState.RELEASE);
                setPincerPosition(frontPincerRight, PincerState.RELEASE);
                break;

            case SWEEP_FRONT:
                setPincerPosition(frontPincerLeft, opposite(tempLeftPincerPosition));
                setPincerPosition(frontPincerRight, opposite(tempRightPincerPosition));
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

    public void setPincerPosition(Servo pincer, PincerState desiredState){
        double releasePosition;
        double gripPosition;
        if (pincer == pincerLeft) {
            releasePosition = PINCERL_CLOSED_POSITION;
            gripPosition = releasePosition + PINCER_GRIP_OFFSET;
        } else if (pincer == pincerRight) {
            releasePosition = PINCERR_CLOSED_POSITION;
            gripPosition = releasePosition - PINCER_GRIP_OFFSET;
        } else if (pincer == frontPincerLeft) {
            gripPosition = FRONT_PINCERL_CLOSED_POSITION;
            releasePosition = gripPosition + FRONT_PINCER_OPEN_OFFSET;
        } else if (pincer == frontPincerRight) {
            gripPosition = FRONT_PINCERR_CLOSED_POSITION;
            releasePosition = gripPosition - FRONT_PINCER_OPEN_OFFSET;
        } else {
            throw new IllegalArgumentException(pincer + " is not a known pincer servo");
        }
        switch (desiredState){
            case GRIP:
                pincer.setPosition(gripPosition);
                break;
            case RELEASE:
                pincer.setPosition(releasePosition);
                break;
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

    private PincerState bumperToPincer(boolean bumper) {
        return bumper ? PincerState.RELEASE : PincerState.GRIP;
    }

    private PincerState opposite(PincerState pincerState) {
        return bumperToPincer(pincerState == PincerState.GRIP);
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
                } else if (desiredState == EffectorState.SWEEP_FRONT) {
                    nextPosition = EffectorState.SWEEP_FRONT;
                }
                break;
            case SWEEP_FRONT:
                if (desiredState == EffectorState.STAGED_INTAKE || desiredState == EffectorState.INTAKE){
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
