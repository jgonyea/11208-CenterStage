package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;



@TeleOp
public class KardiaTeleOp extends OpMode {

    DcMotor liftMotorL;
    DistanceSensor distL;
    DistanceSensor distR;
    Locomotion motionUtil;
    Lift liftUtil;

    @Override
    public void init(){
        liftMotorL = hardwareMap.get(DcMotor.class, "lifeL");
        motionUtil.init();
    }

    @Override
    public void loop(){
        motionUtil.moveRobot(gamepad1, distL, distR);
        liftUtil.moveLift(gamepad2);
    }

    // Calculates power to motors based on GP1 thumbsticks.
    public void locomotion(){




    }
}
