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
    private double positionFired = 0.3767;

    private boolean launched = false;
    private boolean launching = false;
    public boolean wasLaunched() {
        return launched;
    }

    // Configures launcher servo.
    public void init(Servo launcherServo){
        this.droneLauncher = launcherServo;
        this.droneLauncher.setPosition(positionLoaded);
    }

    // Launches the drone.
    public void launch(Gamepad gamepad1, Gamepad gamepad2){
        // Requires both drivers to press X at the same time.
        if (gamepad1.x && gamepad2.x && !launching) {
            this.droneLauncher.setPosition(positionFired);
            this.launched = true;
            this.launching = true;
            this.timer.reset();
        }

        // Return servo to loading position.
        if (this.timer.seconds() > 2) {
            this.droneLauncher.setPosition(positionLoaded);

            if (!(gamepad1.x || gamepad2.x)) {
                this.launching = false;
            }
        }
    }
}
