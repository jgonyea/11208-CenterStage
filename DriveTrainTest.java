package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is a basic TeleOp for testing the DriveTrain class.
 */
@TeleOp
public class DriveTrainTest extends OpMode {
    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;
    DriveTrain drive;
    @Override
    public void init() {
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.MM;

        drive = new DriveTrain();
        drive.init(hardwareMap, distL, distR, distUnit);
    }

    @Override
    public void loop() {
        drive.moveRobot(gamepad1);
    }
}
