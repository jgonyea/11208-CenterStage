/**
 *  Translates robot based on input from gamepad and distanceSensors.
 *  Original power calculation code from https://www.youtube.com/@gavinford8924
 */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DriveTrain {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private DistanceSensor distanceL;
    private DistanceSensor distanceR;


    // Todo: fix these fake values.
    // Optimal distance for robot to be from scoring board.
    private double OPTIMAL_DIST = 10;

    // Minimum difference in distance sensors to each other for auto-correction to engage.
    private double MINIMUM_DIST = 2;

    // Scales approach speed.
    private double APPROACH_POWER_SCALE = 0.25;

    // Configure drivetrain motors.
    public void init(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;

        //this.distanceL = distanceL;
        //this.distanceR = distanceR;

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Move robot based on input from gamepad and distance sensors.
    public void moveRobot(Gamepad gamepad) {

        //double distanceLeft = this.distanceL.getDistance(DistanceUnit.MM);
        //double distanceRight = this.distanceR.getDistance(DistanceUnit.MM);
        double theta;
        double power;
        double turn;

        // Calculate values based on math code from https://www.youtube.com/@gavinford8924
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Locks turning.
            x = 0.0;

            // Approach board until OPTIMAL_DIST.
            //if (distanceLeft < OPTIMAL_DIST || distanceRight < OPTIMAL_DIST){
            //    y = 0.0;
            //} else {
                y = APPROACH_POWER_SCALE * y;
            //}
        }

        theta = Math.atan2(y, x);
        power = Math.hypot(x,y);

        // Automate turning for squaring up to scoring board.
        //if (gamepad.left_bumper) {
        //    turn = calculateTurn(distanceLeft, distanceRight);
        //} else {
            turn = gamepad.right_stick_x;
        //}


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

    // Scales turning values to square robot based on distance sensor input.
    private double calculateTurn(double distanceLeft, double distanceRight){
        // A negative turn value is rotating the robot counter-clockwise.
        // A positive turn value is rotating the robot clockwise.
        double turn = distanceLeft - distanceRight;

        // Don't turn if distance values are both 0.
        if (distanceLeft == 0 && distanceRight == 0) {
            return 0.0;
        }

        // Scale turn.
        turn = turn / Math.max(distanceLeft, distanceRight);

        // todo: Check if we need to account for oscillations?
        // todo: Check if we need to set a minimum turn value.
        return turn;
    }

}
