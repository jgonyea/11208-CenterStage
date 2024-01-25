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
    private int step = 2;



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
                //effector.setDesiredState(Effector.EffectorState.STAGED_INTAKE);
                // todo: set pincer to closed

                //sleep(Effector.STAGED_INTAKE_TIME);
                //effector.setDesiredState(Effector.EffectorState.INTAKE);
                // todo: set pincers to open

                //sleep(Effector.STAGED_INTAKE_TIME * 2);

                //effector.setDesiredState(Effector.EffectorState.DRIVING);

                step++;
            }

            // Locate team prop and maneuver to proper location.
            if (step == 1) { // && effector.getCurrentState() == Effector.EffectorState.DRIVING) {

                step++;
            }


            if (step == 2) {

                step++;

                telemetry.addData("pre-async", "hello");
                telemetry.update();
                robot.followTrajectory(begin);
                robot.turnAsync(Math.toRadians(-135));

                telemetry.addData("post async", "hi");
                telemetry.update();
                while (robot.isBusy()) {
                    robot.update();
                    Pose2d pose = robot.getPoseEstimate();
                    double distRReading = distR.getDistance(distUnit);
                    if (distRReading < minDistanceR) {
                        minDistanceR = distRReading;
                        minDistanceRHeading = pose.getHeading();
                    }
                }

                int propPosition = (int) Math.floor(minDistanceRHeading) - 3;
                telemetry.addData("Detected team prop", propPosition);
                telemetry.update();

                Pose2d currentPose = robot.getPoseEstimate();
                Trajectory spikeLeft = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(8.75, -31.63, 3.124))
                        .build();
                Trajectory spikeCenter = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(11.69, -32.47, 1.54))
                        .build();
                Trajectory spikeRight = robot.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(new Pose2d(14.20, -32.60, 0))
                        .build();

                switch (propPosition) {
                    case 2:
                        robot.followTrajectory(spikeLeft);
                        break;
                    case 1:
                        robot.followTrajectory(spikeCenter);
                        break;
                    case 0:
                        robot.followTrajectory(spikeRight);
                        break;
                    default:
                        break;
                }
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
            //effector.run();
        }
    }
}
