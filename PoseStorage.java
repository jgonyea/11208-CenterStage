package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="View stored pose")
public class PoseStorage extends LinearOpMode {
    private static double x = 0.0;
    private static double y = 0.0;
    private static double h = 0.0;

    public static void setPose(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
        h = pose.getHeading();
    }

    public static Pose2d getPose() {
        return new Pose2d(x, y, h);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("h", h);
        telemetry.addLine();
        telemetry.addData("Pose", getPose());
        telemetry.update();

        waitForStart();
    }
}
