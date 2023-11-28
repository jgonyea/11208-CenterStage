/**
 * Controls the slide lifts via gamepad2.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Lift {

    public DcMotor liftLeft;
    public DcMotor liftRight;
    private int liftTarget;

    // Encoder ticks for one full revolution.
    // Todo: Check tickrates with actual specsheets.
    public double ENCODER_TICKRATE = 28;


    public void init(DcMotor liftLeft, DcMotor liftRight){
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;

        // Number of turns to reach full lift extension.
        // Todo: find the actual value
        liftTarget = 1;

        // Note: Hardware mapping must be performed in OpMode/ LinearOpMode classes.

        // todo: Add lift motor/ encoder config init.
        this.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftLeft.setTargetPosition(encodeTarget(liftTarget));
        this.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Todo: Add right lift.

    }

    public void moveLift(Gamepad gp){
        // Set lift target.
        if (gp.right_trigger > gp.left_trigger) {
            this.liftLeft.setTargetPosition(encodeTarget(liftTarget));
        } else if (gp.right_trigger < gp.left_trigger){
            this.liftLeft.setTargetPosition(0);
        }
        this.liftLeft.setPower(gp.right_trigger - gp.left_trigger);

        // Todo: Add right lift.
    }

    private int encodeTarget(double turns){
        return (int) (ENCODER_TICKRATE * turns);
    }
}

