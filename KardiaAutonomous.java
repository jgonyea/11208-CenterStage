/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;

@Autonomous(name="KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    ElapsedTime autonomousModeTimer = new ElapsedTime();
    ElapsedTime stepTimer = new ElapsedTime();

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Effector effector = new Effector();

    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;

    private double minDistanceL = 10000;
    private double minDistanceLHeading;
    private double minDistanceR = 10000;
    private double minDistanceRHeading;

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

        // Distance Sensors
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.CM;
        robot.setPoseEstimate(new Pose2d(15, -63, Math.toRadians(90)));
            while (!isStarted()) {
                robot.update();
                telemetry.addData("inside loop", "hello");

                Pose2d pose = robot.getPoseEstimate();
                telemetry.addData("Current Pose x", pose.getX());
                telemetry.addData("Current Pose y", pose.getY());
                telemetry.addData("Current Pose h", pose.getHeading());

                telemetry.addData("min L", minDistanceL);
                telemetry.addData("min R", minDistanceR);
                telemetry.addData("min L H", minDistanceLHeading);
                telemetry.addData("min R H", minDistanceRHeading);
                telemetry.update();
            }

        // Pause and wait for driver to press Start.
        autonomousModeTimer.reset();

        Trajectory begin = robot.trajectoryBuilder(robot.getPoseEstimate(), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(15, -42.34, 5.6), Math.toRadians(90))
                .build();

        while (opModeIsActive()) {

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

            // Locate team prop using distance sensor sweep and maneuver to proper location.
            if (step == 1) {
                robot.followTrajectory(begin);
                robot.turnAsync(Math.toRadians(-135));
                while (robot.isBusy()) {
                    robot.update();
                    Pose2d pose = robot.getPoseEstimate();
                    double distRReading = distR.getDistance(distUnit);
                    if (distRReading < minDistanceR) {
                        minDistanceR = distRReading;
                        minDistanceRHeading = pose.getHeading();
                    }
                }

                // Converts teamPropPosition detected radian angle to integer.
                int teamPropPosition = (int) Math.floor(minDistanceRHeading);
                telemetry.addData("Detected team prop", teamPropPosition);
                telemetry.update();

                Pose2d spikeLeftPose = new Pose2d(8.75, -31.63, 3.124);
                Pose2d spikeCenterPose = new Pose2d(11.69, -32.47, 1.54);
                Pose2d spikeRightPose = new Pose2d(14.20, -32.60, 0);
                Pose2d currentPose = robot.getPoseEstimate();

                Trajectory spikeLeft = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikeLeftPose)
                        .build();
                Trajectory spikeCenter = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikeCenterPose)
                        .build();
                Trajectory spikeRight = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikeRightPose)
                        .build();
                Trajectory clearProp;

                switch (teamPropPosition) {
                    case 5: // Left Spike
                        robot.followTrajectory(spikeLeft);
                        clearProp = robot.trajectoryBuilder(spikeLeft.end())
                                .forward(10).build();
                        robot.followTrajectory(clearProp);
                        clearProp = robot.trajectoryBuilder(robot.getPoseEstimate())
                                .lineToLinearHeading(spikeLeft.end()).build();
                        robot.followTrajectory(clearProp);
                        break;
                    case 4: // Center Spike
                        robot.followTrajectory(spikeCenter);
                        clearProp = robot.trajectoryBuilder(spikeCenter.end())
                                .forward(10).build();
                        robot.followTrajectory(clearProp);
                        clearProp = robot.trajectoryBuilder(robot.getPoseEstimate())
                                .lineToLinearHeading(spikeCenter.end()).build();
                        robot.followTrajectory(clearProp);
                        break;
                    case 3: // Right Spike
                        robot.followTrajectory(spikeRight);
                        clearProp = robot.trajectoryBuilder(spikeRight.end())
                                .forward(10).build();
                        break;
                    default:
                        break;
                }

                step++;
            }

            // Place left (purple)
            if (step == 2){
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                // todo: set left pincer to closed
                sleep(Effector.STAGED_INTAKE_TIME * 2);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);
            }

            // Drive to score board
            // Score right (yellow)
            // Drive to parking spot

            // Run modes of robot.
            //effector.run();
        }
    }
}
