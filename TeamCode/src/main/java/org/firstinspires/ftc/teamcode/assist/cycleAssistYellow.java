package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

import java.util.concurrent.TimeUnit;

public class cycleAssistYellow extends cycleAssist{

    //States
    private int state;

    //Times
    private double time = -1;
    private long cycleTime;
    private final double extra = 4.5;
    boolean check;
    private int statePark;

    private Telemetry telemetry;

    //Poses
    private final Pose2d BASKET = new Pose2d(12.5, -9.5, Math.PI/4);

    public cycleAssistYellow(intake i, slideStates s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r, Telemetry t){
        super(i, s, m, sa, r);
        state = 0;
        telemetry = t;
        cycleTime = 0;
        check = false;
        statePark = 0;
    }
    public void reset(){
        state = 0;
        time = -1;
    }

    public boolean preLoad(){
        slide.update();
        in.update();
        drive.update();
        in.drop(1);
        slide.extend(0);

        if(state == 0){ // move to Basket
            movment.moveTo(BASKET, 1.1);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, BASKET);
            double slideToLocation = (20.0-distances[0])*30.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }
            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.25, distances[0], 3)){
                state++;
            }
        } // SlideUp
        if(state == 1){
            movment.moveTo(BASKET, 0.5);
            slide.slideTo(1125.0);
            in.direction(0);
            if(slide.getPos() > 1115.0){
                state++;
            }
        }
        if(state == 2){
            movment.moveTo(new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()), 0.5);
            slide.slideTo(1125.0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()));
            if(checkIfInsideBox(distances[1], 0.35, distances[0], 1.8)){
                state++;
            }
        }
        // deposit
        if(state == 3){
            movment.moveTo(new Pose2d(BASKET.getX()+extra, BASKET.getY()+extra, BASKET.getHeading()), 0.5);
            slide.slideTo(1125.0);
            in.direction(-1);
            if(time == -1){
                runtime.reset();
                time = 0;
            }
            else{
                time = runtime.time(TimeUnit.MILLISECONDS);
            }
            if(time > 450){
                time = -1;
                state ++;
            }
        }
        if(state == 4){
            movment.moveTo(BASKET, 0.5);
            slide.slideTo(1125.0);
            in.direction(0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, BASKET);
            if(checkIfInsideBox(distances[1], 0.25, distances[0], 1.2)){
                state++;
            }
        }

        if(state == 5){ //go back down
            movment.moveTo(BASKET, 0.5);
            slide.slideTo(0);
            in.direction(0);
            if(slide.getPos() < 800){
                state++;
            }
        }
        if(state == 6){
            return true;
        }
        return false;
    }
    public boolean cycle(Pose2d yellowSample, Pose2d Basket, double t, double e, double timeOut, double drop){ //starts and ends with slides down at the basket
        slide.update();
        in.update();
        drive.update();

        if(state == 0){ //ready position

            movment.moveTo(yellowSample);
            Pose2d currentPose = drive.getPoseEstimate();
            slide.slideTo(0);
            in.drop(0.8);
            double[] distances = distanceNumbers(currentPose, yellowSample);
            telemetry.addData("check", check);
            if (checkIfInsideBox(distances[1], 0.5, distances[0], 3) && !check){
                cycleTime = System.currentTimeMillis();
                check = true;
            }
            if((checkIfInsideBox(distances[1], 0.20, distances[0], 1.2) && slide.getPos() < 50) || (check && System.currentTimeMillis() - cycleTime > timeOut)){
                state++;
                check = false;
            }
        }
        if(state == 1){ //intaking

            in.direction(1);
            slide.extend(e);
            in.drop(drop);

            movment.moveToDSB(yellowSample, 0.3);
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
            if(checkIfInsideBox(distances[1], 0.15, distances[0], 2.0) && time > t){
                time = -1;
                state++;
            }
        }
        if(state == 2){ //moving to basket
            movment.moveToRotSpeed(Basket, 0.9, 1.2);

            in.drop(1);

            slide.extend(0);

            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, Basket);
            double slideToLocation = (20.0-distances[0])*30.0;
            if(slideToLocation < 0){
                slideToLocation = 0;
            }

            telemetry.addData("moveToBasket", 2);

            slide.slideTo(slideToLocation);
            if(checkIfInsideBox(distances[1], 0.35, distances[0], 3)){
                state++;
            }
        }
        if(state == 3){ //slide

            movment.moveTo(Basket);

            telemetry.addData("moveToBasket", 2);

            slide.slideTo(1125.0);
            if(slide.getPos() > 1115.0){
                state++;
            }
        }

        if(state == 4){ //forward
            movment.moveTo(new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()), 0.5);
            slide.slideTo(1125.0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()));
            if(checkIfInsideBox(distances[1], 0.2, distances[0], 1.2)){
                state++;
            }
        }
        // deposit
        if(state == 5){
            movment.moveTo(new Pose2d(Basket.getX()+extra, Basket.getY()+extra, Basket.getHeading()), 0.5);
            slide.slideTo(1125.0);
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
        if(state == 6){ //Back
            movment.moveTo(Basket, 0.5);
            slide.slideTo(1125.0);
            in.direction(0);
            Pose2d currentPose = drive.getPoseEstimate();
            double[] distances = distanceNumbers(currentPose, Basket);
            if(checkIfInsideBox(distances[1], 0.2, distances[0], 1.2)){
                state++;
            }
        }
        if(state == 7){ //go back down
            movment.moveTo(Basket);
            slide.slideTo(0);
            in.direction(0);
            if(slide.getPos() < 800){
                state++;
            }
        }
        if(state == 8){
            return true;
        }
        return false;
    }


    public boolean park(){
        Pose2d currentPose = drive.getPoseEstimate();
        slide.update();
        in.update();
        drive.update();
        in.drop(1);
        slide.extend(0);

        if(statePark == 0){
            movment.moveToDSB(new Pose2d(-3, -48, 0), 1.4);

            slide.slideTo(400);
            in.direction(0);
            slide.extend(0);

            double[] distances = distanceNumbers(currentPose, new Pose2d(-3, -48, 0));

            if(checkIfInsideBox(distances[1], 0.5, distances[0], 4)){
                statePark++;
            }
        }
        if(statePark == 1){
            movment.moveTo(new Pose2d(-17, -51, 0), 1.25);

            slide.slideTo(400);
            in.direction(0);
            slide.extend(0);

            double[] distances = distanceNumbers(currentPose, new Pose2d(-17, -51, 0));

        }
        return false;
    }
}