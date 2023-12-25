/**
 *  Translates robot based on input from gamepad and distanceSensors.
 *  Original power calculation code from https://www.youtube.com/@gavinford8924
 */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;


public class DriveTrain {

    private CenterStageDrive centerStageDrive;

    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    private DistanceUnit distanceUnit;


    // Todo: fix these fake values.
    // Optimal distance for robot to be from scoring board.
    private double OPTIMAL_DIST = 10;

    // Minimum difference in distance sensors to each other for auto-correction to engage.
    private double MINIMUM_DIST = 2;

    // Difference between distance values that should set turn to 100%.
    private double MAX_TURN_DIFFERENCE = 300;

    // Scales approach speed.
    private double APPROACH_POWER_SCALE = 0.25;

    // Configure drivetrain motors.
    public void init(HardwareMap hardwareMap,
                     DistanceSensor distanceL, DistanceSensor distanceR, DistanceUnit distanceUnit){
        centerStageDrive = new CenterStageDrive(hardwareMap);

        this.distanceL = distanceL;
        this.distanceR = distanceR;
        this.distanceUnit = distanceUnit;
    }

    // Move robot based on input from gamepad and distance sensors.
    public void moveRobot(Gamepad gamepad) {

        double distanceLeft = this.distanceL.getDistance(this.distanceUnit);
        double distanceRight = this.distanceR.getDistance(this.distanceUnit);
        double theta;
        double power;
        double turn;
        double x;
        double y;

        // Automate turning for squaring up to scoring board.
        if (gamepad.left_bumper) {
            turn = calculateTurn(distanceLeft, distanceRight);
        } else {
            turn = gamepad.right_stick_x;
        }

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Locks turning.
            x = 0.0;

            // Approach board until OPTIMAL_DIST.
            if (distanceLeft < OPTIMAL_DIST || distanceRight < OPTIMAL_DIST){
                y = 0.0;
            } else {
                y = APPROACH_POWER_SCALE * gamepad.left_stick_y;
            }
        } else {
            x = gamepad.left_stick_x;
            y = gamepad.left_stick_y;
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

        // Rescale power if beyond maximum.
        double scale = power + Math.abs(turn);
        if (scale > 1) {
            powerFrontLeft /= scale;
            powerFrontRight /= scale;
            powerRearLeft  /= scale;
            powerRearRight /= scale;
        }

        // Assign power to wheels.
        centerStageDrive.setMotorPowers(powerFrontLeft, powerRearLeft, powerRearRight, powerFrontRight);
        centerStageDrive.update();
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
        turn = Math.max(-1, Math.min(1, turn));

        // todo: Check if we need to account for oscillations?
        // todo: Check if we need to set a minimum turn value.

        return turn;
    }

}
