/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    ElapsedTime gameTimer;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Effector effector = new Effector();

    private boolean first_event = true;
    private boolean second_event = true;
    @Override
    public void runOpMode(){
        // Initialize and configure here.
        gameTimer = new ElapsedTime();

        // Effector hardware mapping.
        armRotatorLeft = hardwareMap.get(Servo.class, "armL");
        armRotatorRight = hardwareMap.get(Servo.class, "armR");
        handActuator = hardwareMap.get(Servo.class, "hand");
        pincerLeft = hardwareMap.get(Servo.class, "pincerL");
        pincerRight = hardwareMap.get(Servo.class, "pincerR");
        wristRotator = hardwareMap.get(Servo.class, "wrist");

        // Todo: write this comment
        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight);
        telemetry.addData("End Effector: ", "Initialized");
        telemetry.update();

        // Pause and wait for driver to press Start.
        waitForStart();
        gameTimer.reset();

        while (opModeIsActive()) {
            telemetry.addData("gameTimer", gameTimer.seconds());
            telemetry.addData("floor(gameTimer)", Math.floor(gameTimer.seconds()));

            if (Math.floor(gameTimer.seconds()) == 3 && first_event) {
                first_event = false;
                effector.setCurrentState(
                        Effector.EffectorState.STAGED_INTAKE, false
                );
            }
            if (Math.floor(gameTimer.seconds()) == 6 && second_event) {
                second_event = false;
                effector.setCurrentState(
                        Effector.EffectorState.STAGED_INTAKE, true
                );
            }

            effector.updateWithoutGamepad();
            effector.moveEffector();
            telemetry.update();
        }
    }
}
