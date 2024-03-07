/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="RedClose-AlignRight")
public class KardiaAutoRedClose extends LinearOpMode {

    CenterStageDrive robot;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Servo frontPincerLeft;
    Servo frontPincerRight;
    Effector effector = new Effector();

    DcMotor liftLeft;
    DcMotor liftRight;
    Lift lift = new Lift();

    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;

    DigitalChannel leftSwitch;
    DigitalChannel centerSwitch;
    DigitalChannel rightSwitchI;
    DigitalChannel rightSwitchII;
    TripleSwitch rightSwitch;

    private enum spike {
        LEFT,
        CENTER,
        RIGHT
    }

    private spike teamPropPosition;

    private double minDistanceR = 10000;
    private double minDistanceRHeading;

    // Increment as each autonomous step proceeds.
    private int step = 0;


    @Override
    public void runOpMode() {

        // All numbers that require manual tuning.
        Pose2d startPose =       new Pose2d(15.435, -60.388, Math.PI * 0.5);
        Pose2d scanningPose =    new Pose2d(16.835, -40.888, 5.6);
        Pose2d spikeLeftPose =   new Pose2d(10.335, -32.558, Math.PI);
        Pose2d spikeCenterPose = new Pose2d(15.125, -32.858, Math.PI * 0.5);
        Pose2d spikeRightPose =  new Pose2d(30.635, -33.388, Math.PI);
        Pose2d scoreCenter =     new Pose2d(41.916, -34.724, Math.PI);
        double scanningTurnAngle = Math.toRadians(-135);
        double scoreOffset = 6.0;
        double parkOffsetY = 24.0;
        double parkOffsetX = -3.0;
        double parkBackup  = 10.0;

        // Switch init must be done first, in case this is a dead auto.
        leftSwitch = hardwareMap.get(DigitalChannel.class, "switch3");
        centerSwitch = hardwareMap.get(DigitalChannel.class, "switch2");
        rightSwitchI = hardwareMap.get(DigitalChannel.class, "parkSwitchI");
        rightSwitchII = hardwareMap.get(DigitalChannel.class, "parkSwitchII");
        rightSwitch = new TripleSwitch(rightSwitchI, rightSwitchII);

        // Check whether this is a dead auto; if so, exit immediately.
        if (leftSwitch.getState()) {
            telemetry.addLine("WARNING! RUNNING DEAD AUTO!");
            telemetry.addLine("Please check the left switch.");
            telemetry.update();
            PoseStorage.setPose(startPose);

            waitForStart();
            return;
        }

        robot = new CenterStageDrive(hardwareMap);

        // Precompute most trajectories.
        TrajectorySequence beginningToScanning = lineTraj(startPose, scanningPose);
        TrajectorySequence scanningTurn = turnTraj(scanningPose, scanningTurnAngle);
        TrajectorySequence leftSpikeTraj = lineTraj(scanningTurn.end(), spikeLeftPose);
        TrajectorySequence centerSpikeTraj = lineTraj(scanningTurn.end(), spikeCenterPose);
        TrajectorySequence rightSpikeTraj = lineTraj(scanningTurn.end(), spikeRightPose);
        TrajectorySequence leftSpikeToScoring = lineTraj(leftSpikeTraj.end(),
                scoreCenter.plus(new Pose2d(0, scoreOffset, 0)));
        TrajectorySequence centerSpikeToScoring = lineTraj(centerSpikeTraj.end(), scoreCenter);
        TrajectorySequence rightSpikeToScoring = lineTraj(rightSpikeTraj.end(),
                scoreCenter.plus(new Pose2d(0, -scoreOffset, 0)));

        // Allocate variables for on-the-fly trajectories.
        TrajectorySequence selectedSpikeTraj = null;
        TrajectorySequence selectedScoreTraj = null;
        TrajectorySequence scoringToParking = null;

        // Configure robot and pick up pixels.
        autoInit(startPose);

        // Autonomous loop.
        while (opModeIsActive()) {

            // Move to scanning position and begin turn.
            if (step == 0) {
                robot.followTrajectorySequenceAsync(beginningToScanning);
                blockDuringMotion();

                robot.followTrajectorySequenceAsync(scanningTurn);

                step++;
            }

            // Locate team prop using distance sensor sweep.
            if (step == 1) {
                if (robot.isBusy()) {
                    Pose2d pose = robot.getPoseEstimate();
                    double distRReading = distR.getDistance(distUnit);
                    if (distRReading < minDistanceR) {
                        minDistanceR = distRReading;
                        minDistanceRHeading = pose.getHeading();
                    }
                } else {
                    // Calculate where the prop was detected.
                    teamPropPosition = calculateSpike(minDistanceRHeading);
                    telemetry.addData("Detected team prop", teamPropPosition.name());
                    telemetryUpdate();

                    step++;
                }
            }

            // Select trajectories based on team prop position.
            if (step == 2) {
                switch (teamPropPosition) {
                    case LEFT:
                        selectedSpikeTraj = leftSpikeTraj;
                        selectedScoreTraj = leftSpikeToScoring;
                        break;

                    case CENTER:
                        selectedSpikeTraj = centerSpikeTraj;
                        selectedScoreTraj = centerSpikeToScoring;
                        break;

                    case RIGHT:
                        selectedSpikeTraj = rightSpikeTraj;
                        selectedScoreTraj = rightSpikeToScoring;
                        break;
                }

                step++;
            }

            // Lower pixels and move to correct spike mark.
            if (step == 3) {
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                robot.followTrajectorySequenceAsync(selectedSpikeTraj);
                blockDuringMotion();

                step++;
            }

            // Place purple pixel.
            if (step == 4) {
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setPincerPosition(pincerLeft, Effector.PincerState.RELEASE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Drive to score board
            if (step == 5) {
                robot.followTrajectorySequenceAsync(selectedScoreTraj);
                blockDuringMotion();

                step++;
            }

            // Prepare effector.
            if (step == 6) {
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.SCORING);

                step++;
            }

            // Approach using distance sensors.
            // Code copied from DriveTrain
            // Todo: Write this only in DriveTrain, call it from Autos
            if (step == 7) {
                double distanceLeft = distL.getDistance(distUnit);
                double distanceRight = distR.getDistance(distUnit);
                double minDiff = Math.min(
                        distanceLeft - DriveTrain.LEFT_SENSOR_OPTIMAL_DIST,
                        distanceRight - DriveTrain.RIGHT_SENSOR_OPTIMAL_DIST
                );

                if (minDiff > DriveTrain.MIN_APPROACH_DIFFERENCE) {
                    double y = -minDiff / DriveTrain.MAX_APPROACH_DIFFERENCE;

                    // Robot is too far. Back up.
                    if (y < 0) {
                        y = Math.min(y, -DriveTrain.MIN_APPROACH_POWER);
                    }

                    // Restrict y values to within MAX_APPROACH_POWER.
                    y = Math.max(-DriveTrain.MAX_APPROACH_POWER, Math.min(DriveTrain.MAX_APPROACH_POWER, y));

                    // Apply power to wheels.
                    robot.setMotorPowers(y, y, y, y);

                } else {
                    // Release manual wheel power.
                    robot.setMotorPowers(0, 0, 0, 0);

                    step++;
                }
            }

            // Place yellow pixel.
            if (step == 8) {
                effector.setPincerPosition(pincerRight, Effector.PincerState.RELEASE);
                sleep(300);
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Park based on switch.
            if (step == 9) {
                Pose2d finalScoreCenter = new Pose2d(
                        robot.getPoseEstimate().getX(), scoreCenter.getY(), scoreCenter.getHeading());
                switch (rightSwitch.getState()) {
                    case UP:
                        scoringToParking = lineTraj(finalScoreCenter,
                                finalScoreCenter.plus(new Pose2d(parkOffsetX, -parkOffsetY, 0)),
                                finalScoreCenter.plus(new Pose2d(parkOffsetX + parkBackup, -parkOffsetY, 0)));
                        break;
                    case DOWN:
                        scoringToParking = lineTraj(finalScoreCenter,
                                finalScoreCenter.plus(new Pose2d(parkOffsetX, parkOffsetY, 0)),
                                finalScoreCenter.plus(new Pose2d(parkOffsetX + parkBackup, parkOffsetY, 0)));
                        break;
                    case MIDDLE:
                        // Leave scoringToParking null.
                        break;
                }

                if (scoringToParking != null) {
                    robot.followTrajectorySequenceAsync(scoringToParking);
                    blockDuringMotion();
                }

                // Finished autonomous routine.
                step = 99;
            }

            robot.update();
            telemetryUpdate();
        }
    }

    private void autoInit(Pose2d startPose) {
        // Reset robot pose estimate.
        robot.setPoseEstimate(startPose);

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
        liftLeft = hardwareMap.get(DcMotor.class, "liftL");
        liftRight = hardwareMap.get(DcMotor.class, "liftR");

        // Distance sensor hardware mapping.
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.CM;

        // Initialize effector and lift.
        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight, frontPincerLeft, frontPincerRight);
        lift.init(liftLeft, liftRight);

        telemetryUpdate();

        // Pick up preload pixels.
        sleep(1200);
        effector.setPincerPosition(frontPincerLeft, Effector.PincerState.GRIP);
        effector.setPincerPosition(frontPincerRight, Effector.PincerState.GRIP);
        sleep(Effector.SWEEP_FRONT_TIME);
        effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
        effector.setPincerPosition(pincerLeft, Effector.PincerState.RELEASE);
        effector.setPincerPosition(pincerRight, Effector.PincerState.RELEASE);
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setDesiredState(Effector.EffectorState.INTAKE);
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setPincerPosition(pincerLeft, Effector.PincerState.GRIP);
        effector.setPincerPosition(pincerRight, Effector.PincerState.GRIP);
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
        effector.setPincerPosition(frontPincerLeft, Effector.PincerState.RELEASE);
        effector.setPincerPosition(frontPincerRight, Effector.PincerState.RELEASE);
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setDesiredState(Effector.EffectorState.DRIVING);
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setDesiredState(Effector.EffectorState.INIT);

        while (opModeInInit()) {
            robot.update();
            telemetryUpdate();
        }

        effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
        sleep(Effector.STAGED_LIFT_TIME);
        effector.setDesiredState(Effector.EffectorState.DRIVING);
    }

    private TrajectorySequence lineTraj(Pose2d begin, Pose2d... morePoses) {
        TrajectorySequenceBuilder builder = robot.trajectorySequenceBuilder(begin);
        for (Pose2d pose : morePoses) {
            builder.lineToLinearHeading(pose);
        }

        return builder.build();
    }

    private TrajectorySequence turnTraj(Pose2d begin, double angle) {
        return robot.trajectorySequenceBuilder(begin)
                .turn(angle)
                .build();
    }

    private void telemetryUpdate() {
        Pose2d pose = robot.getPoseEstimate();
        // Todo: Fix pose "jump" when OpMode stopped.
        if (!isStopRequested()) PoseStorage.setPose(pose);

        telemetry.addData("step #", step);
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

    private spike calculateSpike(double heading) {
        spike detectedSpike = spike.CENTER;

        // Left Spike
        if (heading > 4.84) {
            detectedSpike = spike.LEFT;
        }

        // Center Spike
        // if (heading >= 4.19 && heading <= 4.84) {
        // detectedSpike = spike.CENTER;
        // }

        // Right Spike
        if (heading < 4.19) {
            detectedSpike = spike.RIGHT;
        }

        return detectedSpike;
    }

    private void blockDuringMotion() {
        while (opModeIsActive() && robot.isBusy()) {
            robot.update();
            telemetryUpdate();
        }
    }
}
