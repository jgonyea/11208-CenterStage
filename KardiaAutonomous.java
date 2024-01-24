/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;

@Autonomous(name="KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    ElapsedTime autonomousModeTimer = new ElapsedTime();
    ElapsedTime stepTimer = new ElapsedTime();;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Effector effector = new Effector();

    // todo: add camera and vision processors.

    // Increment as each autonoumous step proceeds.
    private int step = 0;



    @Override
    public void runOpMode(){
        // Initialize and configure here.
        CenterStageDrive robot = new CenterStageDrive(hardwareMap);

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
        autonomousModeTimer.reset();

        robot.setPoseEstimate(new Pose2d(15, -63, 90));

        Trajectory begin = robot.trajectoryBuilder(robot.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(1, 2, 3), 0)
                .build();

        Trajectory nextTraj = robot.trajectoryBuilder(begin.end())
                .splineToLinearHeading(new Pose2d(1, 2, 3), 0)
                .build();

        while (opModeIsActive()) {
            telemetry.addData("Game Time: ", autonomousModeTimer.seconds());

            // todo: fill in missing autonomous programming.

            // Grab pixels from starting position.
            if (step == 0) {
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                // todo: set pincer to closed

                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                // todo: set pincers to open

                sleep(Effector.STAGED_INTAKE_TIME * 2);

                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Locate team prop and maneuver to proper location.
            if (step == 1 && effector.getCurrentState() == Effector.EffectorState.DRIVING) {

                step++;
            }


            if (step == 2) {

                step++;
            }




            //Trajectory traj = robot.trajectoryBuilder(new Pose2d())
            //        .splineTo(new Vector2d(30, 30), 0)
            //        .build();
            //
            //robot.followTrajectory(traj);
            // Place left (purple)
            // Locate April tag for global positioning reorientation?
            // Drive to score board
            // Score right (yellow)
            // Drive to parking spot

            // Run modes of robot.
            effector.run();

            telemetry.update();
        }
    }
}
