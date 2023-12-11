/**
 * Controls launching of drone on the robot.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drone {
    private Servo droneLauncher;
    private ElapsedTime timer = new ElapsedTime();

    private double positionLoaded = 0;
    private double positionFired = 1;
    private boolean launching = false;

    // Configures launcher servo.
    public void init(Servo launcherServo){
        this.droneLauncher = launcherServo;
        this.droneLauncher.setPosition(positionLoaded);
    }

    // Launches the drone.
    public void launch(Gamepad gamepad1, Gamepad gamepad2){
        // Requires both player to press button "Y" and the same time.
        if (gamepad1.x && gamepad2.x && !launching) {
            launching = true;
            this.timer.reset();
        } else {
            // Reset servo position.
            this.droneLauncher.setPosition(positionLoaded);
        }

        // Launch drone and toggle launching flag.
        while (launching) {
            this.droneLauncher.setPosition(positionFired);
            if (this.timer.seconds() > 2) {
                launching = false;
            }
        }
    }
}
