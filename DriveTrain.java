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
    public static final double LEFT_SENSOR_OPTIMAL_DIST = 3.1;
    public static final double RIGHT_SENSOR_OPTIMAL_DIST = 4.15;

    // Minimum speed for auto-align functions.
    public static final double MIN_ALIGN_POWER = 0.1;

    // Minimum difference in heading for auto-correction to engage.
    public static final double MIN_HEADING_DIFFERENCE = 0.01;

    // Difference from target that sets power to 100%.
    public static final double MAX_HEADING_DIFFERENCE = 1.2;

    // Minimum speed for auto-approach.
    public static final double MIN_APPROACH_POWER = 0.07;

    // Minimum difference from OPTIMAL_DIST to engage auto-approach.
    public static final double MIN_APPROACH_DIFFERENCE = 1.0;

    // Difference from OPTIMAL_DIST that should make motors run at 100% backwards.
    public static final double MAX_APPROACH_DIFFERENCE = 100.0;

    // Power level that auto-approach should not exceed.
    public static final double MAX_APPROACH_POWER = 0.4;

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
        centerStageDrive.setPoseEstimate(PoseStorage.pose);

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
        if (gamepad.left_bumper && !gamepad.right_stick_button) {
            turn = calculateTurnPowerFromTarget(
                    centerStageDrive.getPoseEstimate().getHeading(), Math.PI
            );
        }

        // Square to nearest 90 degrees when right stick pressed.
        if (gamepad.right_stick_button && !gamepad.left_bumper) {
            turn = calculateTurnPowerFromTarget(
                    centerStageDrive.getPoseEstimate().getHeading(),
                    nearest90(centerStageDrive.getPoseEstimate().getHeading())
            );
        }

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Approach board until OPTIMAL_DIST.
            double minDiff = Math.min(
                    distanceLeft - LEFT_SENSOR_OPTIMAL_DIST,
                    distanceRight - RIGHT_SENSOR_OPTIMAL_DIST
            );
            if (minDiff > MIN_APPROACH_DIFFERENCE) {
                y = -minDiff / MAX_APPROACH_DIFFERENCE;

                // Robot is too far. Back up.
                if (y < 0) {
                    y = Math.min(y, -MIN_APPROACH_POWER);
                }

                // Restrict y values to within MAX_APPROACH_POWER.
                y = Math.max(-MAX_APPROACH_POWER, Math.min(MAX_APPROACH_POWER, y));

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
        if (gamepad.left_bumper || gamepad.right_bumper || gamepad.right_stick_button) {
            throttle = 1;
        } else {
            throttle = gear / 3.0;
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
     * Calculates turn power based on current and target heading.
     * @param currentHeading
     *  Current heading.
     * @param targetHeading
     *  Target heading.
     * @return double
     *  Automated turn value.
     */
    private double calculateTurnPowerFromTarget(double currentHeading, double targetHeading) {
        double headingDifference = targetHeading - currentHeading;
        if (headingDifference < -Math.PI) {
            headingDifference += Math.PI * 2;
        }
        if (headingDifference > Math.PI) {
            headingDifference -= Math.PI * 2;
        }

        if (Math.abs(headingDifference) >= MIN_HEADING_DIFFERENCE) {
            double turnPower = -headingDifference / MAX_HEADING_DIFFERENCE;
            if (turnPower > 0 && turnPower < MIN_ALIGN_POWER) {
                turnPower = MIN_ALIGN_POWER;
            }
            if (turnPower < 0 && turnPower > -MIN_ALIGN_POWER) {
                turnPower = -MIN_ALIGN_POWER;
            }
            return Math.max(-1, Math.min(1, turnPower));
        } else {
            return 0.0;
        }
    }

    private double nearest90(double heading) {
        if (heading > Math.PI * 0.25 && heading <= Math.PI * 0.75)
            return Math.PI * 0.5;
        if (heading > Math.PI * 0.75 && heading <= Math.PI * 1.25)
            return Math.PI;
        if (heading > Math.PI * 1.25 && heading <= Math.PI * 1.75)
            return Math.PI * 1.5;
        if (heading > Math.PI * 1.75 || heading <= Math.PI * 0.25)
            return 0;

        // Failsafe in case heading is not in [0, 2pi).
        return heading;
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

    public Pose2d getPoseEstimate() {
        return centerStageDrive.getPoseEstimate();
    }

    public void setCurrentThrottleMode(ThrottleMode throttleMode) {
        this.currentThrottleMode = throttleMode;
    }

    public ThrottleMode getCurrentThrottleMode() {
        return this.currentThrottleMode;
    }
}
