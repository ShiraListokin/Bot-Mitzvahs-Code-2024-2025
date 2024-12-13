package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

import java.util.concurrent.TimeUnit;

public class cycleAssistYellow extends cycleAssist{

    //States
    private int state;

    //Times
    private double time = -1;
    private final double extra = 2.2;

    //Poses
    private final Pose2d BASKET = new Pose2d(9.5, -13.5, Math.PI/4);

    public cycleAssistYellow(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r){
        super(i, s, m, sa, r);
        state = 0;
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
            double slideToLocation = (20.0-distances[0])*20.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }
            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
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
        }
        if(state == 2){
            movment.moveTo(new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()));
            slide.slideTo(1150.0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()));
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
                state++;
            }
        }
        // deposit
        if(state == 3){
            movment.moveTo(new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()));
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
        if(state == 4){
            movment.moveTo(BASKET);
            slide.slideTo(1150.0);
            in.direction(0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, BASKET);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
                state++;
            }
        }
        if(state == 5){ //go back down
            movment.moveTo(BASKET);
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
    public boolean cycle(Pose2d yellowSample, Pose2d Basket, double distanceY, double distsanceX, double t, double X2, double Y2, double t2){ //starts and ends with slides down at the basket
        slide.update();
        in.update();
        drive.update();
        slide.linkageTo(0);
        if(state == 0){ //ready position
            Pose2d ready = new Pose2d(yellowSample.getX() - distsanceX, yellowSample.getY() + distanceY, yellowSample.getHeading());
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
            if(time == -1){
                runtime.reset();
                time = 0;
            }
            else{
                time = runtime.time(TimeUnit.MILLISECONDS);
            }
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 2.0) || time > t){
                time = -1;
                state++;
            }
        }
        if(state == 2){ //intaking2
            Pose2d intake2 = new Pose2d(yellowSample.getX() - X2, yellowSample.getY() - Y2, yellowSample.getHeading());
            in.direction(1);
            movment.moveTo(intake2, 0.425);
            Pose2d currentPose = drive.getPoseEstimate();
            slide.slideTo(0);
            double[] distances = distanceNumbers(currentPose, intake2);
            if(time == -1){
                runtime.reset();
                time = 0;
            }
            else{
                time = runtime.time(TimeUnit.MILLISECONDS);
            }
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1.5) || time > t2){
                time = -1;
                state++;
            }
        }

        if(state == 3){ //moving to basket
            movment.moveTo(Basket);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, Basket);
            double slideToLocation = (20.0-distances[0])*20.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }
            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
                state++;
            }
        }
        if(state == 4){ //slide
            movment.moveTo(Basket);
            slide.slideTo(1150.0);
            if(slide.state()[0] > 1125.0){
                state++;
            }
        }

        if(state == 5){ //forward
            movment.moveTo(new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()));
            slide.slideTo(1150.0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()));
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
                state++;
            }
        }
        // deposit
        if(state == 6){
            movment.moveTo(new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()));
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
        if(state == 7){ //Back
            movment.moveTo(Basket);
            slide.slideTo(1150.0);
            in.direction(0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, Basket);
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 1)){
                state++;
            }
        }
        if(state == 8){ //go back down
            movment.moveTo(Basket);
            slide.slideTo(0);
            in.direction(0);
            if(slide.state()[0] < 350){
                state++;
            }
        }
        if(state == 9){
            return true;
        }
        return false;
    }
}