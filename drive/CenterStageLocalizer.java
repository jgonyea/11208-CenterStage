package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||           |
 *    | ||           |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class CenterStageLocalizer extends TwoTrackingWheelLocalizer {
    // https://www.revrobotics.com/rev-11-1271/
    public static double TICKS_PER_REV = 8192;

    // https://openodometry.weebly.com/design.html
    public static double WHEEL_RADIUS = (35.0 / 2.0) / 25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private Encoder parallelEncoder, prpendicEncoder;

    private CenterStageDrive drive;

    public CenterStageLocalizer(HardwareMap hardwareMap, CenterStageDrive drive) {
        super(Arrays.asList(
                // Measured from robot
                new Pose2d(+0.7563, -6.9688, Math.toRadians( 0)), // right
                new Pose2d(-6.7500, -0.2813, Math.toRadians(90))  // front
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveFL"));
        prpendicEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveRR"));

        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        prpendicEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(prpendicEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(prpendicEncoder.getCorrectedVelocity())
        );
    }
}
