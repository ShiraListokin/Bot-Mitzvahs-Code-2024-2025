package org.firstinspires.ftc.teamcode.roadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000.0;
    public static double WHEEL_RADIUS = 0.629921; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -6.72723700787;//-1.98706547460600254; // X is the up and down direction
    public static double PARALLEL_Y = 3.30709;//-5.4002408143126095; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6.63332913386; //6.2209923361138495;
    public static double PERPENDICULAR_Y = -3.30709;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Encoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LSlide"));


        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    /*public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, double PlX, double PlY, double PpX, double PpY) {
        super(Arrays.asList(
                new Pose2d(PlX, PlY, 0),
                new Pose2d(PpX, PpY, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Encoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LSlide"));


        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

     */


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return (drive.getRawExternalHeading());
    }

    @Override
    public Double getHeadingVelocity() {
        return (drive.getExternalHeadingVelocity());
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()),
                encoderTicksToInches(perpendicularEncoder.getRawVelocity())
        );
    }
}