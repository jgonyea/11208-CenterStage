/**
 * Controls the slide lift via gamepad input.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Lift {

    public DcMotor liftLeft;
    public DcMotor liftRight;
    // Number of turns to reach full lift extension.
    private static final double MAX_TURNS = 5.6;

    // Encoder ticks for one full revolution.
    public static final double ENCODER_TICKRATE = 751.8;


    public void init(DcMotor liftLeft, DcMotor liftRight){
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;

        // Note: Hardware mapping must be performed in OpMode classes.

        this.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftLeft.setTargetPosition(0);
        this.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftRight.setTargetPosition(0);
        this.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftRight.setDirection(DcMotor.Direction.REVERSE);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void manualUpdate(Gamepad gp){
        // Calculate target position
        double targetPosition = 0;
        if (gp.left_stick_y > 0) {
            targetPosition = MAX_TURNS;
        } else if (gp.left_stick_y < 0){
            targetPosition = 0;
        }

        // Set lift motors' target position and power
        setLiftTarget(targetPosition, Math.abs(gp.left_stick_y));
    }

    public void setLiftTarget(double targetTurns, double power) {
        this.liftLeft.setTargetPosition(encodeTarget(targetTurns));
        this.liftRight.setTargetPosition(encodeTarget(targetTurns));

        this.liftLeft.setPower(power);
        this.liftRight.setPower(power);
    }

    private int encodeTarget(double turns){
        return (int) (ENCODER_TICKRATE * turns);
    }
}
