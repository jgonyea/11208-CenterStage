package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
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
}
