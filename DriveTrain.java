/**
 *  Translates robot based on input from gamepad1 and distanceSensors.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DriveTrain {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor rearRight;
    DcMotor rearLeft;
    Telemetry telemetry;

    // Todo: fix this fake value.
    double MAX_DIST = 1000;

    // Configure drivetrain motors.
    public void init(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;

        // todo: I don't know which direction they are mounted in.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void moveRobot(Gamepad gamepad1, DistanceSensor distL, DistanceSensor distR) {

        // Calculate values
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x,y);
        double turn;

        // Override manual turning.
        if (gamepad1.right_bumper) {
            turn = Math.max(-1, Math.min(1, (distL.getDistance(DistanceUnit.CM) - distR.getDistance(DistanceUnit.CM)) / MAX_DIST));
        } else {
            turn = gamepad1.right_stick_x;
        }

        // Calculate initial power results to motors.
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(
                Math.abs(sin),
                Math.abs(cos)
        );

        // Set power levels for each wheel.
        double powerFrontLeft = power * (cos / max) + turn;
        double powerFrontRight = power * (sin / max) - turn;
        double powerRearLeft = power * (sin / max) + turn;
        double powerRearRight = power * (cos / max) - turn;

        // Rescale power if beyond maximum.
        double scale = power + Math.abs(turn);
        if (scale > 1) {
            powerFrontLeft /= scale;
            powerFrontRight /= scale;
            powerRearLeft  /= scale;
            powerRearRight /= scale;
        }

        // Assign power to wheels.
        frontRight.setPower(powerFrontRight);
        frontLeft.setPower(powerFrontLeft);
        rearRight.setPower(powerRearRight);
        rearLeft.setPower(powerRearLeft);

        // Todo: Add encoder wheels' metrics.

    }

}
