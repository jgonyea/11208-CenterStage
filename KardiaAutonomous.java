/**
 * Autonomous routine.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="KardiaAutonomous")
public class KardiaAutonomous extends LinearOpMode {

    ElapsedTime gameTimer;
    SampleMecanumDrive mecanumDrive;
    Trajectory someTrajectory;
    Pose2d lastPose;
    @Override
    public void runOpMode(){
        // Initialize and configure here.
        gameTimer = new ElapsedTime();
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        lastPose = new Pose2d(0, 0, 0);
        mecanumDrive.setPoseEstimate(lastPose);
        someTrajectory = mecanumDrive.trajectoryBuilder(lastPose)
                .splineTo(new Vector2d(0, 10), Math.toRadians(45))
                .splineTo(new Vector2d(40, 10),0)
                .build();

        // Pause and wait for driver to press Start.
        waitForStart();
        gameTimer.reset();

        // Todo: Move robot to end-goal destination.
        mecanumDrive.followTrajectory(someTrajectory);

    }
}
