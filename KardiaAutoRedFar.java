/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
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
    Effector effector = new Effector();

    private enum spike {
        LEFT,
        CENTER,
        RIGHT
    }

    private double minDistanceLHeading;
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
        Pose2d spikeLeftPose =   new Pose2d(-34, -34.67, Math.PI);
        Pose2d spikeCenterPose = new Pose2d(-32.31, -36.47, Math.PI / 2);
        Pose2d spikeRightPose =  new Pose2d(-29.8, -37.8, 0);


        Pose2d scoreCenter =     new Pose2d(49, 36, Math.PI);
        Pose2d parking =         new Pose2d(scoreCenter.getX(), scoreCenter.getY() + 20, scoreCenter.getHeading());
        Pose2d spikePose = null;
        Pose2d finalScorePose = null;
        Trajectory intermediateTraj = null;
        Trajectory toCenterSquare = null;
        Trajectory spikeClear = null;
        Trajectory toScoreBoard = null;
        double scoreOffset = 6.0;


        // Build trajectory to scanning position.
        Trajectory beginningToScanning = robot.trajectoryBuilder(startPose, startPose.getHeading())
                .splineToLinearHeading(scanningPose, startPose.getHeading())
                .build();

        // Configure robot.
        autoInit(startPose);


        // Autonomous loop.
        while (opModeIsActive()) {

            // Grab pixels from starting position.
            if (step == 0) {
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                effector.setPincerPosition(pincerLeft, Effector.PINCER_STATE.CLOSED);
                effector.setPincerPosition(pincerRight, Effector.PINCER_STATE.CLOSED);

                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setPincerPosition(pincerLeft, Effector.PINCER_STATE.GRIP);
                effector.setPincerPosition(pincerRight, Effector.PINCER_STATE.GRIP);

                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Debug
            if (step == 98){
                robot.followTrajectory(beginningToScanning);
                step = 99;
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
                        finalScorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() + scoreOffset, scoreCenter.getHeading());
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
                        .forward(10)
                        .build();
                Trajectory clearPropBackward = robot.trajectoryBuilder(clearPropForward.end())
                        .back(10)
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
                effector.setPincerPosition(pincerLeft, Effector.PINCER_STATE.CLOSED);
                sleep(Effector.STAGED_INTAKE_TIME * 2);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;

            }

            // Clears position from placed purple pixel.
            if (step == 3 && finalScorePose != null){
                Pose2d currentPose = robot.getPoseEstimate();
                toCenterSquare = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(scanningPose.vec() ,currentPose.getHeading()))
                        .build();

                robot.followTrajectory(toCenterSquare);
                spikeClear = robot.trajectoryBuilder(toCenterSquare.end())
                        .lineToLinearHeading(new Pose2d(scanningPose.getX(), scanningPose.getY() - 12, Math.PI))
                        .build();
                robot.followTrajectory(spikeClear);
                step ++;
            }

            // Spline maneuver to begin approach.
            if (step == 4){
                Trajectory slider = robot.trajectoryBuilder(spikeClear.end())
                        .splineToLinearHeading(new Pose2d(-50.83, -46.43, Math.PI), Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(-50.83, -17.91, Math.PI), Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(0, -17.91, Math.PI), 0)
                        .build();
                robot.followTrajectory(slider);
                step = 99;
            }


            if (step == 13){
                //toScoreBoard = robot.trajectoryBuilder(currentPose)
                //        .lineToLinearHeading(scorePose)
                //        .build();

                //robot.followTrajectory(toScoreBoard);
            }
            // Score right (yellow) when path is clear
            if (step == 14){
                //lift.setLiftTarget(0, 1);
                //effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                //sleep((long) Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.SCORING);
                sleep(1000);
                effector.setPincerPosition(pincerRight, Effector.PINCER_STATE.CLOSED);
                sleep(300);
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep((long) Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);
                //lift.setLiftTarget(0,1);


                step++;
            }

            // Drive to parking spot
            if (step == 15){
                Trajectory park = robot.trajectoryBuilder(toScoreBoard.end())
                        .lineToLinearHeading(parking)
                        .build();
                robot.followTrajectory(park);

                step = 99;
            }


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

        // Lift hardware mapping.
        lift.init(hardwareMap.get(DcMotor.class, "liftL"),
                  hardwareMap.get(DcMotor.class, "liftR"));

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
            telemetryUpdate();
        }

        // Pause and wait for driver to press Start.
        autonomousModeTimer.reset();

    }

    public void telemetryUpdate(){
        Pose2d pose = robot.getPoseEstimate();
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
}