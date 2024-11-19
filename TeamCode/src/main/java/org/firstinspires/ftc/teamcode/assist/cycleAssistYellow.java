package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

public class cycleAssistYellow {

    intake in;
    slides slide;
    utilMovment movment;
    SampleMecanumDrive drive;
    int state;
    final Pose2d BASKET = new Pose2d();
    public cycleAssistYellow(intake i, slides s, utilMovment m, SampleMecanumDrive sa){
        in = i;
        slide = s;
        movment = m;
        drive = sa;
        state = 0;
    }
    public void reset(){
        state = 0;
    }

    public boolean cycle(Pose2d yellowSample){ //starts and ends with slides down at the basket
        slide.update();
        in.update();
        drive.update();
        if(state == 0){ //ready position
            Pose2d ready = new Pose2d(yellowSample.getX(), yellowSample.getY() + 10, yellowSample.getHeading());
            movment.moveTo(ready);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, ready);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.0)){
                state++;
            }
        }
        if(state == 1){ //intaking
            in.direction(1);
            movment.moveTo(yellowSample);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, yellowSample);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.0)){
                state++;
            }
        }
        if(state == 2){ //moving to basket
            movment.moveTo(new);
        }
        Pose2d ready = new Pose2d(yellowSample.getX(), yellowSample.getX() + 10, yellowSample.getHeading());
        movment.moveTo(ready);
        Pose2d currentPose = drive.getPoseEstimate();
        double[] distances = distanceNumbers(currentPose, ready);
        if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.0)){

        }
        Pose2d currentPose = drive.getPoseEstimate();
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
