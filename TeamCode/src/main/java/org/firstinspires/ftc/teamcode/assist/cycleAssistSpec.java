package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class cycleAssistSpec extends cycleAssist{

    //States
    private int placeState;
    private int pushState;
    private int cycleState;
    private int setUpState;

    //Time
    private double time = -1;

    //Poses:

    //Preload
    private final Pose2d PRELOAD = new Pose2d(20, -0.4, 0);

    //Push
    private final Pose2d BOTTOM_LEFT_CORNER_SPEC1 = new Pose2d(27, -29, 0); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SPEC1 = new Pose2d(42, -30, 0); //TODO fill in
    private final Pose2d TOP_MIDDLE_CORNER_SPEC1 = new Pose2d(42, -42, 0); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC1 = new Pose2d(8, -42, 0); //TODO fill in
    private final Pose2d TOP_RIGHT_CORNER_SPEC1 = new Pose2d(42, -52, 0); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC2 = new Pose2d(8, -52, 0); //TODO fill in


    //Set up
    private final Pose2d TOP_RIGHT_CORNER_SET_UP = new Pose2d(20, -44, -Math.PI/6); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SET_UP = new Pose2d(20, -25, -Math.PI/3); //TODO fill in
    private final Pose2d BOTTOM_LEFT_CORNER_SET_UP = new Pose2d(4, -24, -Math.PI/2); //TODO fill in

    //Park
    private final Pose2d PARK = new Pose2d(4, -43, 0); //TODO fill in

    public cycleAssistSpec(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r) {
        super(i, s, m, sa, r);
        placeState = 0;
        pushState = 0;
        cycleState = 0;
        setUpState = 0;
    }

    //reset functions
    public void resetAll() {
        time = -1;
        placeState = 0;
        pushState = 0;
        cycleState = 0;
        setUpState = 0;
    }

    public void resetPlase(){
        placeState = 0;
    }

    public void resetPush(){
        pushState = 0;
    }

    public void resetCycle(){
        cycleState = 0;
    }

    public void resetSetUp(){
        setUpState = 0;
    }

    protected boolean plaseSpecimin(Pose2d start){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (placeState == 0) { // move to preSet

            //Drive
            movment.moveTo(start, 1.42);
            double[] distances = distanceNumbers(currentPose, start);

            //Slides
            double slideToLocation = 690;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1) && slide.state()[0] > 680) {
                placeState++;
            }
        }

        if (placeState == 1) { // move forward

            //Drive
            Pose2d secondStep = new Pose2d(start.getX() + 6, start.getY(), 0);
            movment.moveTo(secondStep);
            double[] distances = distanceNumbers(currentPose, secondStep);

            //Slides
            double slideToLocation = 685;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 0.5) && slide.state()[0] > 680) {
                placeState++;
            }
        }

        if(placeState == 2){ //Move slides down

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
                placeState++;
            }
        }

        if(placeState == 3){ //Move back

            //Drive
            Pose2d thirdStep = new Pose2d(start.getX() - 5, start.getY(), 0);
            movment.moveToDSB(thirdStep, 1.42);
            double[] distances = distanceNumbers(currentPose, thirdStep);

            //SLides
            double slideToLocation = 640;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                placeState++;
            }
        }

        if(placeState == 4){ //finished
            placeState = 0;
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
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 1) { // Move to top left

            //Drive
            movment.moveToDSB(TOP_LEFT_CORNER_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, TOP_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 2) { // Move to top right

            //Drive
            movment.moveToDSB(TOP_MIDDLE_CORNER_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 3) { // Move to observation zone

            //Drive
            movment.moveToDSB(OBSERVATION_ZONE_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 4) { // Move Back

            //Drive
            movment.moveToDSB(TOP_MIDDLE_CORNER_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 5) { // Move to observation zone

            //Drive
            movment.moveToDSB(TOP_RIGHT_CORNER_SPEC1, 2.0);
            double[] distances = distanceNumbers(currentPose, TOP_RIGHT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                pushState++;
            }
        }

        if (pushState == 6) { // Move to observation zone

            //Drive
            movment.moveToDSB(OBSERVATION_ZONE_SPEC2, 2.0);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC2);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 8)) {
                pushState++;
            }
        }

        if(pushState == 7){
            pushState = 0;
            return true;
        }

        return false;
    }

    public boolean preLoad() {
        boolean completed = plaseSpecimin(PRELOAD);
        return completed;
    }

    public boolean setUP(){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (pushState == 0) { // Move to top right

            //Drive
            movment.moveToDSB(TOP_RIGHT_CORNER_SET_UP, 1.1);
            double[] distances = distanceNumbers(currentPose, TOP_RIGHT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if (pushState == 1) { // Move to top left

            //Drive
            movment.moveToDSB(TOP_LEFT_CORNER_SET_UP, 1.1);
            double[] distances = distanceNumbers(currentPose, TOP_LEFT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if (pushState == 2) { // Move to boddum right

            //Drive
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SET_UP, 1.1);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if(pushState == 3){
            return true;
        }

        return false;
    }

    public boolean cycle(Pose2d DEPOSIT, Pose2d INTAKE_CYCLE){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (cycleState == 0) { // Intake

            //Drive
            movment.moveToDSB(INTAKE_CYCLE, 0.6);
            double[] distances = distanceNumbers(currentPose, INTAKE_CYCLE);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1)) {
                cycleState++;
            }
        }
        if (cycleState == 1) { // moveSlideUp

            //Drive
            movment.moveTo(INTAKE_CYCLE);

            //Slides
            double slideToLocation = 60;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (slide.state()[0] > 50) {
                cycleState++;
            }
        }

        if(cycleState == 2){
            boolean plased = plaseSpecimin(DEPOSIT);
            if(plased){
                cycleState++;
            }
        }
        if(cycleState == 3){

            //Drive
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SET_UP, 1.42);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 5)) {
                cycleState++;
            }
        }

        if(cycleState == 4){
            cycleState = 0;
            return true;
        }
        return false;
    }

    public void park(){
        //update
        slide.update();
        in.update();
        drive.update();

        movment.moveTo(PARK);

        //Slides
        double slideToLocation = 0;
        slide.slideTo(slideToLocation);

        //Linkage
        slide.linkageTo(0);

        //Intake
        in.direction(0);
    }
}