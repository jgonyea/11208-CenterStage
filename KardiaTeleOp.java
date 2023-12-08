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
    DistanceSensor distL;
    DistanceSensor distR;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;

    DcMotor liftMotorLeft;
    DcMotor liftMotorRight;


    DriveTrain drivetrain = new DriveTrain();
    Effector effector = new Effector();
    Lift lift = new Lift();

    @Override
    public void init() {
        telemetry.addData("Stage","Pre-Init");
        // Hardware mapping.

        // Drivetrain hardware mapping.
        frontLeft = hardwareMap.get(DcMotor.class, "driveFL");
        frontRight = hardwareMap.get(DcMotor.class, "driveFR");
        rearLeft = hardwareMap.get(DcMotor.class, "driveRL");
        rearRight = hardwareMap.get(DcMotor.class, "driveRR");

        // Effector hardware mapping.
        armRotatorLeft = hardwareMap.get(Servo.class, "armL");
        armRotatorRight = hardwareMap.get(Servo.class, "armR");
        handActuator = hardwareMap.get(Servo.class, "hand");
        pincerLeft = hardwareMap.get(Servo.class, "pincerL");
        pincerRight = hardwareMap.get(Servo.class, "pincerR");
        wristRotator = hardwareMap.get(Servo.class, "wrist");

        // Lift hardware mapping.
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftL");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftR");

        // Class initializations.
        drivetrain.init(frontLeft, frontRight, rearLeft, rearRight);
        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight);
        lift.init(liftMotorLeft, liftMotorRight);

        // Debug.  Todo: Remove before competition.
        telemetry.addData("Initialization", "Complete");
    }

    @Override
    public void loop() {
        drivetrain.moveRobot(gamepad1, distL, distR);
        lift.moveLift(gamepad2);
        effector.moveEffector(gamepad2);

        // Debug.  Todo: Remove before competition.
        telemetry.addData("LiftL Current Pos: ", liftMotorLeft.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorLeft.getTargetPosition());
    }
}
