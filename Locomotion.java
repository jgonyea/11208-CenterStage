package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Locomotion {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor rearRight;
    DcMotor rearLeft;


    public void init(){

    }
    public void moveRobot(Gamepad gamepad1, DistanceSensor distL, DistanceSensor distR) {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x,y);
        double turn;

        // Override manual turning.
        if (gamepad1.right_bumper) {
            turn = distL.getDistance(DistanceUnit.CM) - distR.getDistance(DistanceUnit.CM);
        } else {
            turn = gamepad1.right_stick_x;
        }


    }

}
