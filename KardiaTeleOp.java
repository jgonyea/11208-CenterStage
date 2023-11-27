/**
 * Driver-controlled routine.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;



@TeleOp(name="TeleOp")
public class KardiaTeleOp extends OpMode {

    DcMotor liftMotorL;
    DistanceSensor distL;
    DistanceSensor distR;
    DriveTrain drive;
    Lift lift;

    @Override
    public void init(){
        liftMotorL = hardwareMap.get(DcMotor.class, "lifeL");
        drive.init();
    }

    @Override
    public void loop(){
        drive.moveRobot(gamepad1, distL, distR);
        lift.moveLift(gamepad2);


        telemetry.addData("Label", "Data");
        telemetry.update();
    }
}
