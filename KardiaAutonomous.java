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

@Autonomous(name="RED-KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    CenterStageDrive robot;

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

    private double minDistanceLHeading;
    private double minDistanceR = 10000;
    private double minDistanceRHeading;

    // Increment as each autonomous step proceeds.
    private int step = 1;


    @Override
    public void runOpMode(){
        // Initialize and configure here.
        robot = new CenterStageDrive(hardwareMap);

        // Positions/ poses.
        Pose2d startPose = new Pose2d(15, -63, Math.toRadians(90));
        Pose2d scanningPose = new Pose2d(15, -47.34, 5.6);
        Pose2d spikeLeftPose = new Pose2d(8.4, -36.67, Math.PI);
        Pose2d spikeCenterPose = new Pose2d(11.69, -32.47, 1.57);
        Pose2d spikeRightPose = new Pose2d(14.20, -32.60, 0);
        Pose2d scoreCenter = new Pose2d(50, -36, 180);
        Pose2d parking = new Pose2d(50, -60, 180); // todo: fix this pose.
        Pose2d spikePose = null;
        Pose2d scorePose = null;
        double scoreOffset = 6.0;


        // Build trajectory to scanning position.
        Trajectory beginningToScanning = robot.trajectoryBuilder(startPose, Math.toRadians(90))
                .splineToLinearHeading(scanningPose, Math.toRadians(90))
                .build();

        // Configure robot.
        autoInit(startPose);


        // Autonomous loop.
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
                robot.followTrajectory(beginningToScanning);
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

                Pose2d currentPose = robot.getPoseEstimate();

                // Set both spike and scoring positions.
                switch (teamPropPosition){
                    case 5:
                        spikePose = spikeLeftPose;
                        scorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() - scoreOffset);
                        break;
                    case 4:
                        spikePose = spikeCenterPose;
                        scorePose = scoreCenter;
                        break;
                    case 3:
                        spikePose = spikeRightPose;
                        scorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() - scoreOffset);
                        break;
                    default:
                        spikePose = spikeCenterPose;
                        scorePose = scoreCenter;
                }
                // Will move to center of spike and push team prop out of the way.
                // todo: Check if this can work by chaining?  If so, this is much easier.
                Trajectory toSpike = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikePose)
                        .build();
                robot.followTrajectory(toSpike);
                Trajectory clearPropForward = robot.trajectoryBuilder(toSpike.end())
                        .forward(10)
                        .build();
                Trajectory clearPropBackward = robot.trajectoryBuilder(clearPropForward.end())
                        .back(11)
                        .build();
                robot.followTrajectory(clearPropForward);
                robot.followTrajectory(clearPropBackward);


                //Trajectory spikeLeft = robot.trajectoryBuilder(currentPose)
                //        .lineToLinearHeading(spikeLeftPose)
                //        .build();
                //Trajectory spikeCenter = robot.trajectoryBuilder(currentPose)
                //        .lineToLinearHeading(spikeCenterPose)
                //        .build();
                //Trajectory spikeRight = robot.trajectoryBuilder(currentPose)
                //        .lineToLinearHeading(spikeRightPose)
                //        .build();

                //Trajectory clearProp;

                //switch (teamPropPosition) {
                //    case 5: // Left Spike
                //        robot.followTrajectory(spikeLeft);
                //        clearProp = robot.trajectoryBuilder(spikeLeft.end())
                //                .forward(10).build();

                //        robot.followTrajectory(clearProp);

                //        clearProp = robot.trajectoryBuilder(robot.getPoseEstimate())
                //                .lineToLinearHeading(spikeLeft.end()).build();
                //        robot.followTrajectory(clearProp);
                //        break;
                //    case 4: // Center Spike
                //        robot.followTrajectory(spikeCenter);
                //        clearProp = robot.trajectoryBuilder(spikeCenter.end())
                //                .forward(10).build();
                //        robot.followTrajectory(clearProp);
                //        clearProp = robot.trajectoryBuilder(robot.getPoseEstimate())
                //                .lineToLinearHeading(spikeCenter.end()).build();
                //        robot.followTrajectory(clearProp);
                //        break;
                //    case 3: // Right Spike
                //        robot.followTrajectory(spikeRight);
                //        // Due to how the robot follows to this point, moving the prop isn't necessary.
                //        break;
                //    default:
                //        break;
                //}

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

                // todo: don't skip 3.
                step++;
            }

            // Drive to score board
            if (step == 3 && scorePose != null){
                Pose2d currentPose = robot.getPoseEstimate();
                Trajectory toScoreBoard = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(scorePose)
                        .build();

                robot.followTrajectoryAsync(toScoreBoard);
                stepTimer.reset();
                while (robot.isBusy() && stepTimer.seconds() < 10) {
                    // todo: Avoid collisions as best as we can.  Does this not move the robot since we're not updating the robot?
                    if (distL.getDistance(distUnit) < 10 && distR.getDistance(distUnit) < 10) {
                        continue;
                    }
                    robot.update();
                }


                // Todo: we might not be where we expect.  Can we update without finishing the previous path?


                step++;
            }

            // Score right (yellow) when path is clear
            if (step == 4 && distL.getDistance(distUnit) > 10 && distR.getDistance(distUnit) > 10){
                // todo: score yellow


                step++;
            }

            // Drive to parking spot
            if (step == 5){
                // todo: drive to parking zone
                // todo: Where is the actual parking?  Can we just strafe away from scoreboard?


                step++;
            }


        }
    }

    private void autoInit(Pose2d startPose){

        // Effector hardware mapping.
        armRotatorLeft = hardwareMap.get(Servo.class, "armL");
        armRotatorRight = hardwareMap.get(Servo.class, "armR");
        handActuator = hardwareMap.get(Servo.class, "hand");
        pincerLeft = hardwareMap.get(Servo.class, "pincerL");
        pincerRight = hardwareMap.get(Servo.class, "pincerR");
        wristRotator = hardwareMap.get(Servo.class, "wrist");

        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight);
        telemetry.addData("End Effector: ", "Initialized");
        telemetry.update();

        // Distance Sensors
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.CM;
        robot.setPoseEstimate(startPose);
        while (!isStarted()) {
            robot.update();
            telemetry.addData("Initialized: ", "Status - Waiting");

            Pose2d pose = robot.getPoseEstimate();
            telemetry.addData("Current Pose x (in): ", Math.floor(pose.getX() * 1000) / 1000);
            telemetry.addData("Current Pose y (in): ", Math.floor(pose.getY() * 1000) / 1000);
            telemetry.addData("Current Pose h (rad): ", Math.floor(pose.getHeading() * 1000) / 1000);
            telemetry.addData("Current Pose h (deg): ", Math.floor(pose.getHeading() * 180 / Math.PI));

            telemetry.addData("DistL (cm): ", distL);
            telemetry.addData("DistR (cm): ", distR);
            telemetry.addData("min R", minDistanceR);
            telemetry.addData("min R H", minDistanceRHeading);
            telemetry.update();
        }

        // Pause and wait for driver to press Start.
        autonomousModeTimer.reset();

    }
}
