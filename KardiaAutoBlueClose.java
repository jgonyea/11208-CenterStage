/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.CenterStageDrive;

@Autonomous(name="BlueClose-AlignLeft")
public class KardiaAutoBlueClose extends LinearOpMode {

    CenterStageDrive robot;

    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;

    DigitalChannel leftSwitch;
    DigitalChannel centerSwitch;
    DigitalChannel rightSwitchI;
    DigitalChannel rightSwitchII;
    TripleSwitch rightSwitch;

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

        leftSwitch = hardwareMap.get(DigitalChannel.class, "switch3");
        centerSwitch = hardwareMap.get(DigitalChannel.class, "switch2");
        rightSwitchI = hardwareMap.get(DigitalChannel.class, "parkSwitchI");
        rightSwitchII = hardwareMap.get(DigitalChannel.class, "parkSwitchII");
        rightSwitch = new TripleSwitch(rightSwitchI, rightSwitchII);

        // Positions/ poses.
        Pose2d startPose =       new Pose2d(12, 63, Math.PI * 1.5);
        Pose2d scanningPose =    new Pose2d(8.6, 43.5, 2.46);
        Pose2d spikeLeftIntermediate = new Pose2d(27.45, 55, 5.37);
        Pose2d spikeLeftPose =   new Pose2d(27.6, 34.67, Math.PI);
        Pose2d spikeCenterPose = new Pose2d(7.56, 36, Math.PI * 1.5);
        Pose2d spikeRightIntermediate = new Pose2d(11.6, 36, Math.PI);
        Pose2d spikeRightPose =  new Pose2d(5.8, 35.8, Math.PI);

        Pose2d scoreCenter =     new Pose2d(40, 36, Math.PI);
        Pose2d parking = null;
        Pose2d spikePose = null;
        Pose2d scorePose = null;
        Trajectory intermediateTraj = null;
        Trajectory toScoreBoard = null;
        spike teamPropPosition = null;
        double scoreOffset = 6.0;


        // Build trajectory to scanning position.
        Trajectory beginningToScanning = robot.trajectoryBuilder(startPose, startPose.getHeading())
                .splineToLinearHeading(scanningPose, startPose.getHeading())
                .build();

        // todo: Configure settings based on switches.
        switch (rightSwitch.getState()) {
            case UP:
                parking = new Pose2d(scoreCenter.getX() - 4, scoreCenter.getY() + 25, scoreCenter.getHeading());
                break;

            case MIDDLE:
                // Do not park.
                break;

            case DOWN:
                parking = new Pose2d(scoreCenter.getX() - 4, scoreCenter.getY() - 25, scoreCenter.getHeading());
                break;
        }

        // Configure robot.
        autoInit(startPose);


        // Autonomous loop.
        while (opModeIsActive()) {

            // Grab pixels from starting position.
            if (step == 0) {

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
                teamPropPosition = calculateSpike(minDistanceRHeading);
                telemetry.addData("Detected team prop", teamPropPosition.name());
                telemetryUpdate();

                Pose2d currentPose = robot.getPoseEstimate();

                Trajectory scanningToLeft = robot.trajectoryBuilder(currentPose)
                        .lineToSplineHeading(spikeLeftIntermediate)
                        .build();
                Trajectory scanningToRight = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(spikeRightIntermediate)
                        .build();

                // Set both spike and scoring positions.
                switch (teamPropPosition){
                    case LEFT:
                        spikePose = spikeLeftPose;
                        intermediateTraj = scanningToLeft;
                        scorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() + scoreOffset + 4, scoreCenter.getHeading());
                        break;
                    case CENTER:
                        spikePose = spikeCenterPose;
                        scorePose = scoreCenter;
                        break;
                    case RIGHT:
                        spikePose = spikeRightPose;
                        intermediateTraj = scanningToRight;
                        scorePose = new Pose2d(scoreCenter.getX(), scoreCenter.getY() - scoreOffset, scoreCenter.getHeading());
                        break;
                    default:
                        spikePose = spikeCenterPose;
                        scorePose = scoreCenter;
                }

                // Will move to center of spike and push team prop out of the way.
                Trajectory toSpike;
                if (intermediateTraj != null) {
                    robot.followTrajectory(intermediateTraj);
                    toSpike = robot.trajectoryBuilder(intermediateTraj.end())
                            .lineToLinearHeading(spikePose)
                            .build();
                } else {
                    toSpike = robot.trajectoryBuilder(currentPose)
                            .lineToLinearHeading(spikePose)
                            .build();
                }

                // Lower pixels to push prop.
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);

                robot.followTrajectory(toSpike);

                step++;
            }

            // Place left (purple)
            if (step == 2) {
                effector.setDesiredState(Effector.EffectorState.INTAKE);
                effector.setPincerPosition(pincerLeft, Effector.PincerState.RELEASE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                sleep(Effector.STAGED_INTAKE_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Drive to score board
            if (step == 3 && scorePose != null){
                Pose2d currentPose = robot.getPoseEstimate();
                toScoreBoard = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(scorePose)
                        .build();

                robot.followTrajectory(toScoreBoard);

                step++;
            }

            // Raise effector arm
            if (step == 4) {
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.SCORING);

                step++;
            }

            // Approach score board using distance sensors
            if (step == 5) {
                double minDiff;
                do {
                    double distanceLeft = distL.getDistance(distUnit);
                    double distanceRight = distR.getDistance(distUnit);
                    minDiff = Math.min(
                            distanceLeft - DriveTrain.LEFT_SENSOR_OPTIMAL_DIST,
                            distanceRight - DriveTrain.RIGHT_SENSOR_OPTIMAL_DIST
                    );
                    double y = -minDiff / DriveTrain.MAX_APPROACH_DIFFERENCE;

                    // Robot is too far. Back up.
                    if (y < 0) {
                        y = Math.min(y, -DriveTrain.MIN_APPROACH_POWER);
                    }

                    // Restrict y values to within MAX_APPROACH_POWER.
                    y = Math.max(-DriveTrain.MAX_APPROACH_POWER, Math.min(DriveTrain.MAX_APPROACH_POWER, y));

                    // Apply power to wheels.
                    robot.setMotorPowers(y, y, y, y);
                    robot.update();

                } while (minDiff > DriveTrain.MIN_APPROACH_DIFFERENCE);

                // Release manual wheel power.
                robot.setMotorPowers(0, 0, 0, 0);
                step++;
            }

            // Score right (yellow) when path is clear
            if (step == 6) {
                effector.setPincerPosition(pincerRight, Effector.PincerState.RELEASE);
                sleep(300);
                effector.setDesiredState(Effector.EffectorState.STAGED_LIFT);
                sleep(Effector.STAGED_LIFT_TIME);
                effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Drive to parking spot
            if (step == 7) {
                if (parking == null) {
                    step = 99;
                } else {
                    Trajectory park = robot.trajectoryBuilder(toScoreBoard.end())
                            .lineToLinearHeading(parking)
                            .build();
                    robot.followTrajectory(park);

                    step = 99;
                }
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

        // Pick up preload pixels.
        sleep(700);
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
        sleep(Effector.STAGED_INTAKE_TIME);
        effector.setDesiredState(Effector.EffectorState.DRIVING);

        while (!isStarted()) {
            robot.update();
            telemetry.addData("Initialized: ", "Status - Waiting");
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
        if (heading > 1.7) {
            detectedSpike = spike.LEFT;
        }
        // Center Spike
        if (heading >= 1.05 && heading <= 1.7) {
            detectedSpike = spike.CENTER;
        }
        // Right Spike
        if (heading < 1.05 ){
            detectedSpike = spike.RIGHT;
        }

        return detectedSpike;
    }
}
