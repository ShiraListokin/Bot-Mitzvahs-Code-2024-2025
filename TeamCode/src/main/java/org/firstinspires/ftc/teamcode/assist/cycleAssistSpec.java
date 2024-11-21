package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

import java.util.concurrent.TimeUnit;

public class cycleAssistSpec {

    private intake in;
    private slides slide;
    private utilMovment movment;
    private SampleMecanumDrive drive;
    private int state;
    private final Pose2d PRELOAD = new Pose2d(20, -0.4, 0);
    private final Pose2d PRELOAD2 = new Pose2d(27, -0.4, 0);
    private final Pose2d PRELOAD3 = new Pose2d(15, -0.4, 0);
    private double time = -1;
    ElapsedTime runtime;

    public cycleAssistSpec(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r) {
        in = i;
        slide = s;
        movment = m;
        drive = sa;
        state = 0;
        runtime = r;
    }

    public void reset() {
        state = 0;
        time = -1;
    }
//TODO start at 17mm

    public boolean preLoad() {
        slide.update();
        in.update();
        drive.update();
        in.direction(1);

        if (state == 0) { // move to preload
            movment.moveTo(PRELOAD);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, PRELOAD);
            double slideToLocation = 690;
            slide.linkageTo(0.4);
            slide.slideTo(slideToLocation);
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] > 680) {
                state++;
            }
        }
        if (state == 1) { // move to preload
            movment.moveTo(PRELOAD2);
            double slideToLocation = 690;
            slide.slideTo(slideToLocation);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, PRELOAD2);
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] > 680) {
                state++;
            }
        }
        if(state == 2){
            movment.moveTo(PRELOAD2);
            double slideToLocation = 640;
            slide.slideTo(slideToLocation);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, PRELOAD2);
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] < 660) {
                state++;
            }
        }
        if(state == 3){
            movment.moveTo(PRELOAD3);
            double slideToLocation = 640;
            slide.slideTo(slideToLocation);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, PRELOAD3);
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5)) {
                state++;
            }
            slide.linkageTo(0);
        }
        if(state == 4){
            movment.moveTo(PRELOAD3);
            slide.slideTo(0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, PRELOAD3);
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] < 10) {
                state++;
            }
        }
        if(state == 5){
            return true;
        }
        return false;
    }

    protected double[] distanceNumbers(Pose2d pi, Pose2d pf){
        double xi = pi.getX();
        double yi = pi.getY();
        double thetai = pi.getHeading();

        double xf = pf.getX();
        double yf = pf.getY();
        double thetaf = pf.getHeading();

        double deltaX = xf-xi;
        double deltaY = yf-yi;

        double angleInBetween = angleBetween(thetaf, thetai);
        double distance = Math.hypot(deltaX, deltaY);
        return new double[] {distance, angleInBetween};//distance then angle
    }
    protected boolean checkIfInsideBox(double angle, double rotPrec, double dist, double transPrec){
        return ((angle < rotPrec) && (dist < transPrec));
    }
    protected static double angleBetween(double angle1, double angle2) {
        // Normalize the angles to be within 0 to 2π
        angle1 = angle1 % (2 * Math.PI);
        angle2 = angle2 % (2 * Math.PI);

        // Calculate the difference
        double diff = Math.abs(angle1 - angle2);

        // Ensure the angle is the smallest possible
        if (diff > Math.PI) {
            diff = (2 * Math.PI) - diff;
        }

        return diff;
    }

}