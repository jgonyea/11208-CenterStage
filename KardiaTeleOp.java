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
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor rearRight;
    DcMotor rearLeft;
    DcMotor liftMotorL;
    DcMotor liftMotorR;
    DistanceSensor distL;
    DistanceSensor distR;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handRotator;
    Servo leftActuator;
    Servo rightActuator;
    Servo wristActuator;

    DriveTrain drivetrain = new DriveTrain();
    Effector effector = new Effector();
    Lift lift = new Lift();

    @Override
    public void init() {
        telemetry.addData("Stage","Pre-Init");
        // Hardware mapping.
        // Lift
        liftMotorL = hardwareMap.get(DcMotor.class, "liftL");
        liftMotorR = hardwareMap.get(DcMotor.class, "liftR");

        // Effector
        //armRotatorLeft = hardwareMap.get(Servo.class, "armL");
        //armRotatorRight = hardwareMap.get(Servo.class, "armR");
        //handRotator = hardwareMap.get(Servo.class, "hand");
        //leftActuator = hardwareMap.get(Servo.class, "gripL");
        //rightActuator = hardwareMap.get(Servo.class, "gripR");
        //wristActuator = hardwareMap.get(Servo.class, "wrist");


        // Drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFL");
        frontRight = hardwareMap.get(DcMotor.class, "driveFR");
        rearLeft = hardwareMap.get(DcMotor.class, "driveRL");
        rearRight = hardwareMap.get(DcMotor.class, "driveRR");

        // Class initializations.
        lift.init(liftMotorL, liftMotorR);
        drivetrain.init(frontLeft, frontRight, rearLeft, rearRight);  // Todo: Do we need a full robot class to pass around the data fully?
        //effector.init(armRotatorLeft, armRotatorRight, wristActuator, handRotator, leftActuator, rightActuator);


        // Debug.  Todo: Remove before competition.
        telemetry.addData("Initialization", "Complete");
        telemetry.addData("LiftL Target", liftMotorL.getTargetPosition());
        telemetry.addData("LiftL Mode: ", liftMotorL.getMode());
        telemetry.addData("LiftL Current Pos: ", liftMotorL.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorL.getTargetPosition());
        telemetry.addData("Motor NoPower Type", liftMotorL.getZeroPowerBehavior());

        telemetry.addData("Stage","Post-Init");
    }

    @Override
    public void loop() {
        telemetry.addData("Stage", "Loop");
        drivetrain.moveRobot(gamepad1, distL, distR);
        lift.moveLift(gamepad2);
        int liftPosition = liftMotorL.getCurrentPosition();
        effector.moveEffector(gamepad2, liftPosition);
        //rearRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


        // Debug.  Todo: Remove before competition.
        telemetry.addData("1-LTrig", gamepad1.left_trigger);
        telemetry.addData("1-RTrig", gamepad1.right_trigger);
        telemetry.addData("Trigger Diff: ", gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("LiftL Power: ", liftMotorL.getPower());
        telemetry.addData("LiftL Mode: ", liftMotorL.getMode());
        telemetry.addData("LiftL Current Pos: ", liftMotorL.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorL.getTargetPosition());
        telemetry.addData("Motor NoPower Type", liftMotorL.getZeroPowerBehavior());
        telemetry.addData("RR Power", rearRight.getPower());
    }
}
