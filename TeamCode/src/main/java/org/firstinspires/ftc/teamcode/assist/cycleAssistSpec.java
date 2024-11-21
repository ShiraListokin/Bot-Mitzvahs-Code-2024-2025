package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

import java.util.concurrent.TimeUnit;

public class cycleAssistSpec extends cycleAssist{

    //States
    private int subStatePlace;
    private int pushState;

    //Time
    private double time = -1;

    //Poses:

    //Preload
    private final Pose2d PRELOAD = new Pose2d(20, -0.4, 0);

    //Push
    private final Pose2d BOTTOM_LEFT_CORNER_SPEC1 = new Pose2d(0, 0, 0); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SPEC1 = new Pose2d(0, 0, 0); //TODO fill in
    private final Pose2d TOP_RIGHT_CORNER_SPEC1 = new Pose2d(0, 0, 0); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC1 = new Pose2d(0, 0, 0); //TODO fill in

    public cycleAssistSpec(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r) {
        super(i, s, m, sa, r);
        subStatePlace = 0;
        pushState = 0;
    }

    //reset functions
    public void resetAll() {
        time = -1;
        subStatePlace = 0;
        pushState = 0;
    }

    public void resetPlase(){
        subStatePlace = 0;
    }

    public void resetPush(){
        pushState = 0;
    }

    protected boolean plaseSpecimin(Pose2d start){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (subStatePlace == 0) { // move to preSet

            //Drive
            movment.moveTo(start);
            double[] distances = distanceNumbers(currentPose, start);

            //Slides
            double slideToLocation = 690;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] > 680) {
                subStatePlace++;
            }
        }

        if (subStatePlace == 1) { // move forward

            //Drive
            Pose2d secondStep = new Pose2d(start.getX() + 7, start.getY(), 0);
            movment.moveTo(secondStep);
            double[] distances = distanceNumbers(currentPose, secondStep);

            //Slides
            double slideToLocation = 690;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] > 680) {
                subStatePlace++;
            }
        }

        if(subStatePlace == 2){ //Move slides down

            //Drive
            Pose2d secondStep = new Pose2d(start.getX() + 7, start.getY(), 0);
            movment.moveTo(secondStep);
            double[] distances = distanceNumbers(currentPose, secondStep);

            //Slides
            double slideToLocation = 640;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] < 660) {
                subStatePlace++;
            }
        }

        if(subStatePlace == 3){ //Move back

            //Drive
            Pose2d thirdStep = new Pose2d(start.getX() - 5, start.getY(), 0);
            movment.moveTo(thirdStep);
            double[] distances = distanceNumbers(currentPose, thirdStep);

            //SLides
            double slideToLocation = 640;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5)) {
                subStatePlace++;
            }
        }

        if(subStatePlace == 4){ //Move Down

            //Drive
            Pose2d thirdStep = new Pose2d(start.getX() - 5, start.getY(), 0);
            movment.moveTo(thirdStep);
            double[] distances = distanceNumbers(currentPose, thirdStep);

            //SLides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] < 10) { //TODO change the down requirement to move onto the next state
                subStatePlace++;
            }
        }

        if(subStatePlace == 5){ //finished
            return true;
        }

        return false;
    }

    public boolean pushSpec(){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (pushState == 0) { // Move to bottom left

            //Drive
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SPEC1, 0.9);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 2)) {
                pushState++;
            }
        }

        if (pushState == 1) { // Move to top left

            //Drive
            movment.moveToDSB(TOP_LEFT_CORNER_SPEC1, 0.9);
            double[] distances = distanceNumbers(currentPose, TOP_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 2)) {
                pushState++;
            }
        }

        if (pushState == 2) { // Move to top right

            //Drive
            movment.moveToDSB(TOP_RIGHT_CORNER_SPEC1, 0.9);
            double[] distances = distanceNumbers(currentPose, TOP_RIGHT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 2)) {
                pushState++;
            }
        }

        if (pushState == 3) { // Move to observation zone

            //Drive
            movment.moveToDSB(OBSERVATION_ZONE_SPEC1, 0.9);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 2)) {
                pushState++;
            }
        }

        if(pushState == 4){
            return true;
        }

        return false; //TODO return actual value
    }

    public boolean preLoad() {
        boolean completed = plaseSpecimin(PRELOAD);
        return completed;
    }

    public boolean setUP(){

        return false;
    }

    public boolean cycle(){
        return false;
    }
}