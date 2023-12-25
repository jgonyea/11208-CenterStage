package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is a basic TeleOp for testing the DriveTrain class.
 */
@TeleOp
public class DriveTrainTest extends OpMode {
    DriveTrain drive = new DriveTrain();
    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.moveRobotWithoutDistance(gamepad1);
    }
}
