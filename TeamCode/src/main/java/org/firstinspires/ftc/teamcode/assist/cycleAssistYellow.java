package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

import java.util.concurrent.TimeUnit;

public class cycleAssistYellow {

    private intake in;
    private slides slide;
    private utilMovment movment;
    private SampleMecanumDrive drive;
    private int state;
    private final Pose2d BASKET = new Pose2d(10, -13, Math.PI/4);
    private double time = -1;
    ElapsedTime runtime;
    public cycleAssistYellow(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r){
        in = i;
        slide = s;
        movment = m;
        drive = sa;
        state = 0;
        runtime = r;
    }
    public void reset(){
        state = 0;
        time = -1;
    }

    public boolean preLoad(){
        slide.update();
        in.update();
        drive.update();
        slide.linkageTo(0);
        if(state == 0){ // move to Basket
            movment.moveTo(BASKET);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, BASKET);
            double slideToLocation = (20.0-distances[0])*15.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }
            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 0.5)){
                state++;
            }
        } // SlideUp
        if(state == 1){
            movment.moveTo(BASKET);
            slide.slideTo(1150.0);
            in.direction(0);
            if(slide.state()[0] > 1125.0){
                state++;
            }
        } // deposit
        if(state == 2){
            movment.moveTo(BASKET);
            slide.slideTo(1150.0);
            in.direction(-1);
            if(time == -1){
                runtime.reset();
                time = 0;
            }
            else{
                time = runtime.time(TimeUnit.MILLISECONDS);
            }
            if(time > 450){
                state ++;
            }
        }
        if(state == 3){ //go back down
            movment.moveTo(BASKET);
            slide.slideTo(0);
            in.direction(0);
            if(slide.state()[0] < 300){
                state++;
            }
        }
        if(state == 4){
            return true;
        }
        return false;
    }
    public boolean cycle(Pose2d yellowSample, Pose2d Basket, double distance){ //starts and ends with slides down at the basket
        slide.update();
        in.update();
        drive.update();
        slide.linkageTo(0);
        if(state == 0){ //ready position
            Pose2d ready = new Pose2d(yellowSample.getX(), yellowSample.getY() + distance, yellowSample.getHeading());
            movment.moveTo(ready);
            Pose2d currentPose = drive.getPoseEstimate();
            slide.slideTo(0);
            double[] distances = distanceNumbers(currentPose, ready);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.0)){
                state++;
            }
        }
        if(state == 1){ //intaking
            in.direction(1);
            movment.moveTo(yellowSample, 0.5);
            Pose2d currentPose = drive.getPoseEstimate();
            slide.slideTo(0);
            double[] distances = distanceNumbers(currentPose, yellowSample);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.0)){
                state++;
            }
        }
        if(state == 2){ //moving to basket
            movment.moveTo(Basket);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, Basket);
            double slideToLocation = (20.0-distances[0])*15.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }
            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], .75)){
                state++;
            }
        }
        if(state == 3){ //sliding
            movment.moveTo(Basket);
            slide.slideTo(1150.0);
            in.direction(0);
            if(slide.state()[0] > 1125.0){
                state++;
            }
        }
        if(state == 4){ //depositing
            movment.moveTo(Basket);
            slide.slideTo(1150.0);
            in.direction(-1);
            if(time == -1){
                runtime.reset();
                time = 0;
            }
            else{
                time = runtime.time(TimeUnit.MILLISECONDS);
            }
            if(time > 450){
                state ++;
            }
        }
        if(state == 5){ //go back down
            movment.moveTo(Basket);
            slide.slideTo(0);
            in.direction(0);
            if(slide.state()[0] < 350){
                state++;
            }
        }
        if(state == 6){
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
