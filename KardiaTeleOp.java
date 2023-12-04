/**
 * Driver-controlled routine.
 */
package org.firstinspires.ftc.teamcode.teamcode11208;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="KardiaTeleOp")
public class KardiaTeleOp extends OpMode {

    DcMotor liftMotorL;
    DcMotor liftMotorR;
    DistanceSensor distL;
    DistanceSensor distR;
    DriveTrain drivetrain;
    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handRotator;
    Servo leftActuator;
    Servo rightActuator;
    Effector effector;
    Lift lift = new Lift();

    @Override
    public void init() {
        // Hardware mapping.
        liftMotorL = hardwareMap.get(DcMotor.class, "liftL");
        liftMotorR = hardwareMap.get(DcMotor.class, "liftR");

        armRotatorLeft = hardwareMap.get(Servo.class, "armL");
        armRotatorRight = hardwareMap.get(Servo.class, "armR");
        handRotator = hardwareMap.get(Servo.class, "hand");
        leftActuator = hardwareMap.get(Servo.class, "gripL");
        rightActuator = hardwareMap.get(Servo.class, "gripR");


        // Class initializations.
        lift.init(liftMotorL, liftMotorR);
        effector.init(armRotatorLeft, armRotatorRight, handRotator, leftActuator, rightActuator);
        //drivetrain.init();  // Todo: Do we need a full robot class to pass around the data fully?


        // Debug.  Todo: Remove before competition.
        telemetry.addData("Initialization", "Complete");
        telemetry.addData("LiftL Target", liftMotorL.getTargetPosition());
        telemetry.addData("LiftL Mode: ", liftMotorL.getMode());
        telemetry.addData("LiftL Current Pos: ", liftMotorL.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorL.getTargetPosition());
        telemetry.addData("Motor NoPower Type", liftMotorL.getZeroPowerBehavior());
    }

    @Override
    public void loop() {
        //drivetrain.moveRobot(gamepad1, distL, distR);
        lift.moveLift(gamepad1);  // gamepad2 triggers are used by moveEffector
        effector.moveEffector(gamepad2);


        // Debug.  Todo: Remove before competition.
        telemetry.addData("1-LTrig", gamepad1.left_trigger);
        telemetry.addData("1-RTrig", gamepad1.right_trigger);
        telemetry.addData("Trigger Diff: ", gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("LiftL Power: ", liftMotorL.getPower());
        telemetry.addData("LiftL Mode: ", liftMotorL.getMode());
        telemetry.addData("LiftL Current Pos: ", liftMotorL.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorL.getTargetPosition());
        telemetry.addData("Motor NoPower Type", liftMotorL.getZeroPowerBehavior());
    }
}
