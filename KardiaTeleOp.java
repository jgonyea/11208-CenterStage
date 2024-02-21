/**
 * Driver-controlled routine.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="KardiaTeleOp")
public class KardiaTeleOp extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor rearRight;
    DcMotor rearLeft;
    DistanceSensor distL;
    DistanceSensor distR;
    DistanceUnit distUnit;

    Servo armRotatorLeft;
    Servo armRotatorRight;
    Servo handActuator;
    Servo pincerLeft;
    Servo pincerRight;
    Servo wristRotator;
    Servo frontPincerLeft;
    Servo frontPincerRight;

    Servo launcher;

    DcMotor liftMotorLeft;
    DcMotor liftMotorRight;

    DriveTrain drivetrain = new DriveTrain();
    Drone drone = new Drone();
    Effector effector = new Effector();
    Lift lift = new Lift();

    private boolean is_start_pressed = false;

    @Override
    public void init() {
        telemetry.addData("Stage","Pre-Init");
        // Hardware mapping.

        // Distance Sensors
        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distUnit = DistanceUnit.MM;

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
        frontPincerLeft = hardwareMap.get(Servo.class, "frontpL");
        frontPincerRight = hardwareMap.get(Servo.class, "frontpR");

        // Lift hardware mapping.
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftL");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftR");

        // Launcher hardware mapping.
        launcher = hardwareMap.get(Servo.class, "launcher");

        // Class initializations.
        drivetrain.init(hardwareMap,
                        distL, distR, distUnit);
        is_start_pressed = false;
        telemetry.addData("Drivetrain: ", "Initialized");
        drone.init(launcher);
        telemetry.addData("Drone Launcher: ", "Initialized");
        effector.init(armRotatorLeft, armRotatorRight, wristRotator, handActuator, pincerLeft, pincerRight, frontPincerLeft, frontPincerRight);
        telemetry.addData("End Effector: ", "Initialized");
        lift.init(liftMotorLeft, liftMotorRight);
        telemetry.addData("Lift: ", "Initialized");


        telemetry.addData("Robot Initialization", "Complete");
        telemetry.addData("Current Effector State: ", effector.getCurrentState().name());
    }

    @Override
    public void loop() {
        drivetrain.moveRobot(gamepad1);
        drone.launch(gamepad1, gamepad2);
        lift.manualUpdate(gamepad2);
        effector.manualUpdate(gamepad2);

        // Drivetrain throttle mode switch
        if (gamepad1.start && !is_start_pressed) {
            is_start_pressed = true;
            if (drivetrain.getCurrentThrottleMode() == DriveTrain.ThrottleMode.ALL_GEARS) {
                drivetrain.setCurrentThrottleMode(DriveTrain.ThrottleMode.SKIP_SECOND_GEAR);
            } else {
                drivetrain.setCurrentThrottleMode(DriveTrain.ThrottleMode.ALL_GEARS);
            }
        }
        if (!gamepad1.start) {
            is_start_pressed = false;
        }

        // Debug.  Todo: Remove before competition.
        telemetry.addData("LiftL Current Pos: ", liftMotorLeft.getCurrentPosition());
        telemetry.addData("LiftL Target: ", liftMotorLeft.getTargetPosition());
        telemetry.addData("Current Effector State: ", effector.getCurrentState().name());
        telemetry.addData("distL", distL.getDistance(distUnit));
        telemetry.addData("distR", distR.getDistance(distUnit));
        telemetry.addData("Throttle Gear: ", drivetrain.getGear());
        telemetry.addData("Throttle Mode: ", drivetrain.getCurrentThrottleMode().name());
    }
}
