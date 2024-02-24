/**
 *  Translates robot based on input from gamepad and distanceSensors.
 *  Original power calculation code from https://www.youtube.com/@gavinford8924
 */
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Heading difference that should make motors turn at 100%.
    private final double MAX_HEADING_DIFFERENCE = 1.5;

    // Time to wait before re-enabling turn correction.
    private final long MANUAL_TURN_DELAY = 150;

    public enum ThrottleMode {
        ALL_GEARS, SKIP_SECOND_GEAR
    }

    private ThrottleMode currentThrottleMode;

    private boolean isDownPressed;
    private boolean isUpPressed;
    private boolean isYPressed;
    private int gear = 2;

    private double targetHeading;
    private ElapsedTime manualTurnTimer = new ElapsedTime();

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
        targetHeading = centerStageDrive.getPoseEstimate().getHeading();
        manualTurnTimer.reset();
    }

    // Move robot based on input from gamepad and distance sensors.
    public void moveRobot(Gamepad gamepad) {

        double distanceLeft = this.distanceL.getDistance(this.distanceUnit);
        double distanceRight = this.distanceR.getDistance(this.distanceUnit);
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double turn = 0.0;
        double theta;
        double power;

        //create "gears" for the drive train
        if (gamepad.right_trigger == 1 && !isUpPressed) {
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
        if (gamepad.right_trigger < 1) {
            isUpPressed = false;
        }

        if (gamepad.left_trigger == 1 && !isDownPressed) {
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
        if (gamepad.left_trigger < 1) {
            isDownPressed = false;
        }

        // Don't allow gearing to affect the auto-squaring / auto-approach.
        if (!gamepad.left_bumper && !gamepad.right_bumper) {
            throttle = gear / 3.0;
        } else {
            throttle = 0.7;
        }

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
            y = DPAD_POWER;
        }
        if (dpadState == 0b0010) {
            x = -DPAD_POWER;
            y = 0.0;
        }
        if (dpadState == 0b0100) {
            x = 0.0;
            y = -DPAD_POWER;
        }
        if (dpadState == 0b1000) {
            x = DPAD_POWER;
            y = 0.0;
        }

        // Automate turning for squaring up to scoring board.
        if (gamepad.left_bumper) {
            turn = calculateTurnPowerFromDistance(distanceLeft, distanceRight);
        }

        // Automate approaching the scoring board.
        if (gamepad.right_bumper) {
            // Approach board until OPTIMAL_DIST.
            double avgDist = (distanceLeft + distanceRight) / 2;
            if (absdiff(avgDist, OPTIMAL_DIST) > MINIMUM_DIST) {
                y = (OPTIMAL_DIST - avgDist) / MAX_DRIVE_DIFFERENCE;

                // Robot is too far. Back up.
                if (y < 0) {
                    y = Math.min(y, -MINIMUM_POWER);
                }

                // Restrict y values between -1 and 1.
                y = constrain(y);

            }
        }

        // Only turn using right stick when bumpers not pressed.
        if (!gamepad.left_bumper && !gamepad.right_bumper) {
            turn = throttle * -gamepad.right_stick_x;
        }

        // Flip target heading when Y pressed.
        if (gamepad.y && !isYPressed) {
            isYPressed = true;
            targetHeading = (targetHeading + Math.PI) % (Math.PI * 2);
        }
        if (!gamepad.y) {
            isYPressed = false;
        }

        // "Snap" to nearest 90 degree heading when right stick pressed.
        if (gamepad.right_stick_button && !isYPressed) {
            targetHeading = nearest90(centerStageDrive.getPoseEstimate().getHeading());
        }

        // Reset timer to allow for manual turns.
        if (turn != 0) {
            manualTurnTimer.reset();
        }

        // Only approach target heading after a manual turn.
        if (manualTurnTimer.milliseconds() < MANUAL_TURN_DELAY) {
            targetHeading = centerStageDrive.getPoseEstimate().getHeading();
        } else {
            turn = calculateTurnPowerFromTarget(centerStageDrive.getPoseEstimate().getHeading(), targetHeading);
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
        double powerFrontLeft  = power * (cos / max);
        double powerFrontRight = power * (sin / max);
        double powerRearLeft   = power * (sin / max);
        double powerRearRight  = power * (cos / max);

        // Rescale power if beyond maximum.
        double scale = power + Math.abs(turn);
        if (scale > 1) {
            powerFrontLeft  /= scale;
            powerFrontRight /= scale;
            powerRearLeft   /= scale;
            powerRearRight  /= scale;
        }

        // Scale power by throttle.
        powerFrontLeft  *= throttle;
        powerFrontRight *= throttle;
        powerRearLeft   *= throttle;
        powerRearRight  *= throttle;

        // Add turn.
        powerFrontLeft  -= turn;
        powerFrontRight += turn;
        powerRearLeft   -= turn;
        powerRearRight  += turn;

        // Assign power to wheels.
        centerStageDrive.setMotorPowers(powerFrontLeft, powerRearLeft, powerRearRight, powerFrontRight);
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
    private double calculateTurnPowerFromDistance(double distanceLeft, double distanceRight){
        if (absdiff(distanceLeft, distanceRight) < MINIMUM_DIST) {
            return 0;
        }

        // A negative turn value is rotating the robot clockwise.
        // A positive turn value is rotating the robot counter-clockwise.
        double turn = (distanceLeft - distanceRight) / MAX_TURN_DIFFERENCE;

        // Limit turn.
        turn = constrain(turn);


        if (turn > 0) {
            turn = Math.min(turn, MINIMUM_POWER);
        }
        if (turn < 0) {
            turn = Math.max(turn, -MINIMUM_POWER);
        }

        return turn;
    }

    private double calculateTurnPowerFromTarget(double currentHeading, double targetHeading) {
        double headingDifference = targetHeading - currentHeading;
        if (headingDifference < -Math.PI) {
            headingDifference += Math.PI * 2;
        }
        if (headingDifference > Math.PI) {
            headingDifference -= Math.PI * 2;
        }

        return constrain(headingDifference / MAX_HEADING_DIFFERENCE);
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

    private double absdiff(double a, double b) {
        return Math.abs(b - a);
    }

    private double constrain(double power) {
        return Math.min(1, Math.max(-1, power));
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
