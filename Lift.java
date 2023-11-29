/**
 * Controls the slide lifts via gamepad2.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Lift {

    public DcMotor liftLeft;
    public DcMotor liftRight;
    private double liftTarget;

    // Encoder ticks for one full revolution.
    public double ENCODER_TICKRATE = 384.5;


    public void init(DcMotor liftLeft, DcMotor liftRight){
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;

        // Number of turns to reach full lift extension.
        liftTarget = 5.0;

        // Note: Hardware mapping must be performed in OpMode/ LinearOpMode classes.

        this.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftLeft.setTargetPosition(encodeTarget(liftTarget));
        this.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftRight.setTargetPosition(encodeTarget(liftTarget));
        this.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftRight.setDirection(DcMotor.Direction.REVERSE);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveLift(Gamepad gp){
        // Set lift target.
        if (gp.right_trigger > gp.left_trigger) {
            this.liftLeft.setTargetPosition(encodeTarget(liftTarget));
            this.liftRight.setTargetPosition(encodeTarget(liftTarget));
        } else if (gp.right_trigger < gp.left_trigger){
            this.liftLeft.setTargetPosition(0);
            this.liftRight.setTargetPosition(0);
        }

        // Set power for Lift motors
        this.liftLeft.setPower(gp.right_trigger - gp.left_trigger);
        this.liftRight.setPower(gp.right_trigger - gp.left_trigger);

    }

    private int encodeTarget(double turns){
        return (int) (ENCODER_TICKRATE * turns);
    }
}

