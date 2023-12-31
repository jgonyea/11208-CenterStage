/**
 *  Translates robot based on input from gamepad and distanceSensors.
 *  Original power calculation code from https://www.youtube.com/@gavinford8924
 */
package org.firstinspires.ftc.teamcode.teamcode11208;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DriveTrain {

    private double throttle = 1;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    private DistanceUnit distanceUnit;


    // Todo: fix these fake values.
    // Optimal distance for robot to be from scoring board.
    private double OPTIMAL_DIST = 70;

    // Minimum difference in distance sensors to each other for auto-correction to engage.
    private double MINIMUM_DIST = 5;

    // Difference between distance values that should set turn to 100%.
    private double MAX_TURN_DIFFERENCE = 200;

    // Distance from OPTIMAL_DIST that should make motors run at 100% backwards.
    private double MAX_DRIVE_DIFFERENCE = 500;

    // Scales approach speed.
    private double APPROACH_POWER_SCALE = 0.25;

    private boolean isDownPressed;
    private boolean isUpPressed;
    private static final double gearThree = 1;
    private static final double gearTwo = .5;
    private static final double gearOne = .25;
    private int gear = 3;

    // Configure drivetrain motors.
    public void init(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight,
                     DistanceSensor distanceL, DistanceSensor distanceR, DistanceUnit distanceUnit){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;

        this.distanceL = distanceL;
        this.distanceR = distanceR;
        this.distanceUnit = distanceUnit;

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

        double distanceLeft = this.distanceL.getDistance(this.distanceUnit);
        double distanceRight = this.distanceR.getDistance(this.distanceUnit);
        double theta;
        double power;

        // Calculate values based on math code from https://www.youtube.com/@gavinford8924
        double x = -gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        double turn = -gamepad.right_stick_x;

        // Automate turning for squaring up to scoring board.
        if (gamepad.left_bumper) {
            turn = calculateTurn(distanceLeft, distanceRight);
        }

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Locks turning.
            x = 0.0;
            turn = 0.0;

            // Approach board until OPTIMAL_DIST.
            if (distanceLeft < OPTIMAL_DIST || distanceRight < OPTIMAL_DIST){
                y = 0.0;
            } else {
                double difference = ((distanceLeft + distanceRight) / 2) - OPTIMAL_DIST;
                y = Math.max(0.1, Math.min(1, difference / MAX_DRIVE_DIFFERENCE));
            }
        }

        // Calculate values based on math code from https://www.youtube.com/@gavinford8924
        theta = Math.atan2(y, x);
        power = Math.hypot(x,y);

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

        //create "gears" for the drive train
        if (gamepad.dpad_up && !isUpPressed) {
            isUpPressed = true;
            if (gear < 3) {
                gear++;
            }
        }
        if (!gamepad.dpad_up) {
            isUpPressed = false;
        }

        if (gamepad.dpad_down && !isDownPressed) {
            isDownPressed = true;
            if (gear > 1) {
                gear--;
            }
        }
        if (!gamepad.dpad_down) {
            isDownPressed = false;
        }

        // Don't allow gearing to affect the auto-squaring/ auto-approach.
        if (!gamepad.left_bumper && !gamepad.right_bumper) {
            throttle = gear / 3.0;
        } else {
            throttle = 0.7;
        }

        // Rescale power if beyond maximum.
        double scale = power + Math.abs(turn);
        if (scale > 1) {
            powerFrontLeft /= scale;
            powerFrontRight /= scale;
            powerRearLeft  /= scale;
            powerRearRight /= scale;
        }

        // Assign power to wheels.
        frontRight.setPower(powerFrontRight * throttle);
        frontLeft.setPower(powerFrontLeft * throttle);
        rearRight.setPower(powerRearRight * throttle);
        rearLeft.setPower(powerRearLeft * throttle);

    }

    // Scales turning values to square robot based on distance sensor input.
    private double calculateTurn(double distanceLeft, double distanceRight){
        if (Math.abs(distanceLeft - distanceRight) < MINIMUM_DIST) {
            return 0;
        }

        // A negative turn value is rotating the robot counter-clockwise.
        // A positive turn value is rotating the robot clockwise.
        double turn = (distanceLeft - distanceRight) / MAX_TURN_DIFFERENCE;

        // Limit turn.
        turn = Math.max(-0.5, Math.min(0.5, turn));

        return turn;
    }

    public double getThrottle(){
        return this.throttle;
    }
}
