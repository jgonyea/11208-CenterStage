/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;

@Autonomous(name="RedFar-AlignLeft")
public class KardiaAutoRedFar extends LinearOpMode {

    CenterStageDrive robot;
    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;

    ElapsedTime autonomousModeTimer = new ElapsedTime();
    ElapsedTime stepTimer = new ElapsedTime();

    Lift lift = new Lift();

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Servo frontPincerLeft;
    Servo frontPincerRight;
    Effector effector = new Effector();

    private enum spike {
        LEFT,
        CENTER,
        RIGHT
    }

    private double minDistanceR = 10000;
    private double minDistanceRHeading;

    // Increment as each autonomous step proceeds.
    private int step = 0;


    @Override
    public void runOpMode(){
        // Initialize and configure here.
        robot = new CenterStageDrive(hardwareMap);

        // Positions/ poses.
        Pose2d startPose =       new Pose2d(-36, -63, Math.PI / 2);
        Pose2d scanningPose =    new Pose2d(-32.6, -43.5, 5.60);
        Pose2d spikeLeftPose =   new Pose2d(-33, -34.67, Math.PI);
        Pose2d spikeCenterPose = new Pose2d(-36.21, -21.33, Math.PI * 1.5);
        Pose2d spikeRightPose =  new Pose2d(-32.8, -37.8, 0);


        Vector2d afterPurple =   new Vector2d(-33.26, -15.6);
        Pose2d preScoreTraj =    new Pose2d(-32.26, -14.6, Math.PI);
        Pose2d scoreTrajPose2 =  new Pose2d(32.74, -15.6, Math.PI);
        Pose2d scoreCenter =     new Pose2d(45.894, -37.109, Math.PI);
        Pose2d spikePose = null;
        Pose2d finalScorePose = null;
        double scoreOffset = 6.0;


        // Build trajectory to scanning position.
        Trajectory beginningToScanning = robot.trajectoryBuilder(startPose, startPose.getHeading())
                .splineToLinearHeading(scanningPose, startPose.getHeading())
                .build();

        // Configure robot.
        autoInit(startPose);


        // Autonomous loop.
        while (opModeIsActive()) {

            // Debug
            if (step == 98){
                robot.followTrajectory(beginningToScanning);
                step = 99;
            }

            // Grab pixels from starting position.
            if (step == 0) {
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                effector.setPincerPosition(pincerLeft, Effector.PincerState.RELEASE);
                effector.setPincerPosition(pincerRight, Effector.PincerState.RELEASE);

                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setPincerPosition(pincerLeft, Effector.PincerState.GRIP);
                effector.setPincerPosition(pincerRight, Effector.PincerState.GRIP);
                effector.setPincerPosition(frontPincerLeft, Effector.PincerState.RELEASE);
                effector.setPincerPosition(frontPincerRight, Effector.PincerState.RELEASE);

                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
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
                    telemetryUpdate();
                }

                // Converts teamPropPosition detected radian angle to integer.
                spike teamPropPosition = calculateSpike(minDistanceRHeading);
                telemetry.addData("Detected team prop", teamPropPosition.name());
                telemetryUpdate();

                Pose2d currentPose = robot.getPoseEstimate();

                // Set both spike and scoring positions.
                switch (teamPropPosition){
                    case LEFT:
                        spikePose = spikeLeftPose;
                        finalScorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() + scoreOffset + 4, scoreCenter.getHeading());
                        break;
                    case CENTER:
                        spikePose = spikeCenterPose;
                        finalScorePose = scoreCenter;
                        break;
                    case RIGHT:
                        spikePose = spikeRightPose;
                        finalScorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() - scoreOffset, scoreCenter.getHeading());
                        break;
                    default:
                        spikePose = spikeCenterPose;
                        finalScorePose = scoreCenter;
                }

                // Will move to center of spike and push team prop out of the way.
                Trajectory toSpike;
                toSpike = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikePose)
                        .build();


                Trajectory clearPropForward = robot.trajectoryBuilder(toSpike.end())
                        .forward(6)
                        .build();
                Trajectory clearPropBackward = robot.trajectoryBuilder(clearPropForward.end())
                        .back(6)
                        .build();

                robot.followTrajectory(toSpike);
                robot.followTrajectory(clearPropForward);
                robot.followTrajectory(clearPropBackward);

                step++;
            }

            // Place left (purple)
            if (step == 2){
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                effector.setPincerPosition(pincerLeft, Effector.PincerState.RELEASE);
                sleep(Effector.STAGED_INTAKE_TIME * 2);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;

            }

            // Clears position from placed purple pixel.
            if (step == 3 && finalScorePose != null){
                Pose2d currentPose = robot.getPoseEstimate();
                driveToPose(new Pose2d(afterPurple, currentPose.getHeading()));
                step ++;
            }

            // Spline maneuver to begin approach.
            if (step == 4){
                driveToPose(preScoreTraj);
                driveToPose(scoreTrajPose2);
                step++;
            }


            // Wait for distance sensors to clear
            if (step == 5) {
                lift.setLiftTarget(1.2, 1);
                // Todo: what condition should go here
                //if (distL and distR are far enough) {
                    step++;
                //}
            }

            // Drive to score board
            if (step == 6 && finalScorePose != null){
                driveToPose(finalScorePose);
                step++;
            }

            // Score right (yellow)
            if (step == 7) {
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.SCORING);
                sleep(1000);
                effector.setPincerPosition(pincerRight, Effector.PincerState.RELEASE);
                sleep(300);
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);
                lift.setLiftTarget(0, 1);

                step = 99;
            }

            robot.update();
            if (step == 99){
                telemetryUpdate();
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
        frontPincerLeft = hardwareMap.get(Servo.class, "frontpL");
        frontPincerRight = hardwareMap.get(Servo.class, "frontpR");

        // Lift hardware mapping.
        lift.init(hardwareMap.get(DcMotor.class, "liftL"),
                  hardwareMap.get(DcMotor.class, "liftR"));

        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight, frontPincerLeft, frontPincerRight);
        telemetry.addData("End Effector: ", "Initialized");
        telemetry.update();

        // Distance Sensors
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.CM;

        robot.setPoseEstimate(startPose);

        // Close front pincers
        effector.setPincerPosition(frontPincerLeft, Effector.PincerState.GRIP);
        effector.setPincerPosition(frontPincerRight, Effector.PincerState.GRIP);

        while (!isStarted()) {
            robot.update();
            telemetry.addData("Initialized: ", "Status - Waiting");

            Pose2d pose = robot.getPoseEstimate();
            telemetryUpdate();
        }

        // Reset timer and begin autonomous.
        autonomousModeTimer.reset();

    }

    public void telemetryUpdate(){
        Pose2d pose = robot.getPoseEstimate();
        PoseStorage.pose = pose;
        telemetry.addData("Current Pose x (in): ", Math.floor(pose.getX() * 1000) / 1000);
        telemetry.addData("Current Pose y (in): ", Math.floor(pose.getY() * 1000) / 1000);
        telemetry.addData("Current Pose h (rad): ", Math.floor(pose.getHeading() * 1000) / 1000);
        telemetry.addData("Current Pose h (deg): ", Math.floor(pose.getHeading() * 180 / Math.PI));

        telemetry.addData("DistL (cm): ", distL.getDistance(distUnit));
        telemetry.addData("DistR (cm): ", distR.getDistance(distUnit));
        telemetry.addData("min R", minDistanceR);
        telemetry.addData("min R H", minDistanceRHeading);
        telemetry.update();
    }

    private spike calculateSpike(double heading){
        spike detectedSpike = spike.CENTER;

        // Left Spike
        if (heading > 4.84) {
            detectedSpike = spike.LEFT;
        }
        // Center Spike
        if (heading >= 4.19 && heading <= 4.84) {
            detectedSpike = spike.CENTER;
        }
        // Right Spike
        if (heading < 4.19 ){
            detectedSpike = spike.RIGHT;
        }

        return detectedSpike;
    }

    public void driveToPose(Pose2d toPose) {
        Trajectory tempTrajectory = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineToLinearHeading(toPose)
                .build();
        robot.followTrajectory(tempTrajectory);
    }
}
