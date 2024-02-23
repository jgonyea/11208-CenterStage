/**
 *  Translates robot based on input from gamepad and distanceSensors.
 *  Original power calculation code from https://www.youtube.com/@gavinford8924
 */
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;


public class DriveTrain {

    private double throttle = 1;

    private CenterStageDrive centerStageDrive;

    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    private DistanceUnit distanceUnit;
    private ArraySmoother averagerL;
    private ArraySmoother averagerR;
    private static final int SMOOTHING_LENGTH = 3;

    // Optimal distance for robot to be from scoring board.
    private final double OPTIMAL_DIST = 70;

    // Minimum difference in distance sensors for auto-correction to engage.
    private final double MINIMUM_DIST = 10;

    // Difference between distance values that should set turn to 100%.
    private final double MAX_TURN_DIFFERENCE = 200;

    // Distance from OPTIMAL_DIST that should make motors run at 100% backwards.
    private final double MAX_DRIVE_DIFFERENCE = 500;

    // Minimum speed for align and approach.
    private final double MINIMUM_POWER = 0.3;

    // Power level set by dpad when overriding left stick.
    private final double DPAD_POWER = 1.0;

    public enum ThrottleMode {
        ALL_GEARS, SKIP_SECOND_GEAR
    }

    private ThrottleMode currentThrottleMode;

    private boolean isDownPressed;
    private boolean isUpPressed;
    private boolean isYPressed;
    private int gear = 2;

    // Configure drivetrain motors.
    public void init(HardwareMap hardwareMap,
                     DistanceSensor distanceL, DistanceSensor distanceR, DistanceUnit distanceUnit){
        this.centerStageDrive = new CenterStageDrive(hardwareMap);
        centerStageDrive.setPoseEstimate(new Pose2d(0.0, 0.0, 0.0));

        this.distanceL = distanceL;
        this.distanceR = distanceR;
        this.distanceUnit = distanceUnit;
        this.averagerL = new ArraySmoother(SMOOTHING_LENGTH);
        this.averagerR = new ArraySmoother(SMOOTHING_LENGTH);

        currentThrottleMode = ThrottleMode.ALL_GEARS;
    }

    // Move robot based on input from gamepad and distance sensors.
    public void moveRobot(Gamepad gamepad) {

        double distanceLeft = this.distanceL.getDistance(this.distanceUnit);
        double distanceRight = this.distanceR.getDistance(this.distanceUnit);
        double theta;
        double power;

        // Calculate values based on math code from https://www.youtube.com/@gavinford8924
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;

        // Get dpad state as one integer
        int dpadState = 0;
        if (gamepad.dpad_up)
            dpadState |= 0b0001;
        if (gamepad.dpad_left)
            dpadState |= 0b0010;
        if (gamepad.dpad_down)
            dpadState |= 0b0100;
        if (gamepad.dpad_right)
            dpadState |= 0b1000;

        // Override left stick if dpad pressed
        if (dpadState == 0b0001) {
            x = 0.0;
            y = -DPAD_POWER;
        }
        if (dpadState == 0b0010) {
            x = DPAD_POWER;
            y = 0.0;
        }
        if (dpadState == 0b0100) {
            x = 0.0;
            y = DPAD_POWER;
        }
        if (dpadState == 0b1000) {
            x = -DPAD_POWER;
            y = 0.0;
        }

        // Automate turning for squaring up to scoring board.
        if (gamepad.left_bumper) {
            turn = calculateTurn(distanceLeft, distanceRight);
        }

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Approach board until OPTIMAL_DIST.
            double avgDist = (distanceLeft + distanceRight) / 2;
            if (Math.abs(avgDist - OPTIMAL_DIST) > MINIMUM_DIST) {
                y = (OPTIMAL_DIST - avgDist) / MAX_DRIVE_DIFFERENCE;

                // Robot is too far. Back up.
                if (y < 0) {
                    y = Math.min(y, -MINIMUM_POWER);
                }

                // Restrict y values between -1 and 1.
                y = Math.max(-1, Math.min(1, y));

            }
        }

        // Calculate values based on math code from https://www.youtube.com/@gavinford8924
        theta = Math.atan2(y, x);
        power = Math.hypot(x, y);

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
        if (gamepad.right_trigger > 0.5 && !isUpPressed) {
            isUpPressed = true;
            if (gear < 3) {
                switch (currentThrottleMode) {
                    case ALL_GEARS:
                        gear++;
                        break;
                    case SKIP_SECOND_GEAR:
                        gear = 3;
                        break;
                }
            }
        }
        if (gamepad.right_trigger < 0.5) {
            isUpPressed = false;
        }

        if (gamepad.left_trigger > 0.5 && !isDownPressed) {
            isDownPressed = true;
            if (gear > 1) {
                switch (currentThrottleMode) {
                    case ALL_GEARS:
                        gear--;
                        break;
                    case SKIP_SECOND_GEAR:
                        gear = 1;
                        break;
                }
            }
        }
        if (gamepad.left_trigger < 0.5) {
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

        // Scale power by throttle.
        powerFrontLeft  *= throttle;
        powerFrontRight *= throttle;
        powerRearLeft   *= throttle;
        powerRearRight  *= throttle;

        // Turn 180deg when Y pressed.
        if (gamepad.y && !isYPressed) {
            isYPressed = true;
            // See https://learnroadrunner.com/advanced.html#_180%C2%B0-turn-direction
            centerStageDrive.turnAsync(Math.PI + 1e-6);
        }
        if (!gamepad.y) {
            isYPressed = false;
        }

        // Assign power to wheels.
        if (!centerStageDrive.isBusy()) {
            centerStageDrive.setMotorPowers(powerFrontLeft, powerRearLeft, powerRearRight, powerFrontRight);
        }
        centerStageDrive.update();
    }

    /**
     * Scales turning values to square robot based on distance sensor input.
     * @param distanceLeft
     *  Left distance.
     * @param distanceRight
     *  Right distance.
     * @return double
     *  Automated turn value.
     */
    private double calculateTurn(double distanceLeft, double distanceRight){
        if (Math.abs(distanceLeft - distanceRight) < MINIMUM_DIST) {
            return 0;
        }

        // A negative turn value is rotating the robot counter-clockwise.
        // A positive turn value is rotating the robot clockwise.
        double turn = (distanceRight - distanceLeft) / MAX_TURN_DIFFERENCE;

        // Limit turn.
        turn = Math.max(-1, Math.min(1, turn));


        if (turn > 0) {
            turn = Math.min(turn, MINIMUM_POWER);
        }
        if (turn < 0) {
            turn = Math.max(turn, -MINIMUM_POWER);
        }

        return turn;
    }

    /**
     * Getter for robot's current gear.
     * @return int gear
     *  Current gear
     */
    public double getGear(){
        return this.gear;
    }

    /**
     * Getter for current smoothed value of left distance sensor.
     * @return double
     *  Smoothed left distance sensor values.
     */
    public double getSmoothDistL() {
        return averagerL.getSmoothedValue();
    }

    /**
     * Getter for current smoothed value of right distance sensor.
     * @return double
     *  Smoothed right distance sensor values.
     */
    public double getSmoothDistR() {
        return averagerR.getSmoothedValue();
    }

    public void setCurrentThrottleMode(ThrottleMode throttleMode) {
        this.currentThrottleMode = throttleMode;
    }

    public ThrottleMode getCurrentThrottleMode() {
        return this.currentThrottleMode;
    }
}
