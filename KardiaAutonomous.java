/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    ElapsedTime gameTimer;
    @Override
    public void runOpMode(){
        // Initialize and configure here.
        gameTimer = new ElapsedTime();

        // Pause and wait for driver to press Start.
        waitForStart();
        gameTimer.reset();

        // Autonomous routine here.
        while (opModeIsActive()) {
            if (gameTimer.seconds() > 30.0){
                requestOpModeStop();  // Todo: Is this correct?
            }

            // Todo: Move robot to end-goal destination.
        }

        telemetry.addData("Label", "Data");

    }
}
