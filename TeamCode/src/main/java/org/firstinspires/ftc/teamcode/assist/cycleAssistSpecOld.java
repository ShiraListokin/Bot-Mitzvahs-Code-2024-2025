package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class cycleAssistSpecOld extends cycleAssist{

    //Telemetry
    private Telemetry telemetry;

    //States
    private int placeState;
    private int pushState;
    private int cycleState;
    private int setUpState;

    //Time
    private double time = -1;

    //Poses:

    //Preload
    private final Pose2d PRELOAD = new Pose2d(19.15, -0.4, 0);

    //Push
    private final Pose2d BOTTOM_LEFT_CORNER_SPEC1 = new Pose2d(27, -29, 0); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SPEC1 = new Pose2d(46, -30, 0); //TODO fill in
    private final Pose2d TOP_MIDDLE_CORNER_SPEC1 = new Pose2d(46, -42, 0); //TODO fill in
    private final Pose2d TOP_RIGHT_CORNER_SPEC1 = new Pose2d(46, -52, 0); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC1 = new Pose2d(12, -42, 0); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC2 = new Pose2d(12, -52, 0); //TODO fill in
    private final Pose2d IntakeSet = new Pose2d(15, -41.5, Math.PI); //TODO fill in
    private final Pose2d INTAKE2 = new Pose2d(5, -41.5, Math.PI); //TODO fill in
    private final Pose2d DEPOSIT2 = new Pose2d(20.5, 0, 0);

    //Set up
    private final Pose2d TOP_RIGHT_CORNER_SET_UP = new Pose2d(20, -44, -Math.PI/6); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SET_UP = new Pose2d(20, -25, -Math.PI/3); //TODO fill in
    private final Pose2d BOTTOM_LEFT_CORNER_SET_UP = new Pose2d(3, -28, -Math.PI/2); //TODO fill in

    //Set up 2

    //Park
    private final Pose2d PARK = new Pose2d(8, -38, 0); //TODO fill in

    public cycleAssistSpecOld(intake i, slides s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r, Telemetry t) {
        super(i, s, m, sa, r);
        placeState = 0;
        pushState = 0;
        cycleState = 0;
        setUpState = 0;
        telemetry = t;
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

    protected boolean plaseSpecimin(Pose2d start, double hight){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (placeState == 0) { // move to preSet

            //Drive
            movment.moveTo(start, 0.85);
            double[] distances = distanceNumbers(currentPose, start);

            //Slides
            double slideToLocation = hight;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.325);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slide.state()[0] > hight-8) {
                placeState++;
                runtime.reset();
            }
        }

        if (placeState == 1) { // move forward

            //Drive
            Pose2d secondStep = new Pose2d(start.getX() + 7, start.getY(), 0);
            movment.moveTo(secondStep, 0.5);
            double[] distances = distanceNumbers(currentPose, secondStep);

            //Slides
            double slideToLocation = hight;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if ((checkIfInsideBox(distances[1], .25, distances[0], 1) && slide.state()[0] > hight-8) || runtime.time() > 1) {
                placeState++;
                runtime.reset();
            }
        }

        if(placeState == 2){ //Move slides down

            //Drive
            Pose2d secondStep = new Pose2d(start.getX() + 10, start.getY(), 0);
            movment.moveTo(secondStep, 0.5);
            double[] distances = distanceNumbers(currentPose, secondStep);

            //Slides
            double slideToLocation = hight-82;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0.4);

            //Intake
            in.direction(1);

            //Check
            if ((checkIfInsideBox(distances[1], 0.3, distances[0], 1.5) && slide.state()[0] < hight-70)|| runtime.time() > 1.5) {
                placeState++;
            }
        }
        if(placeState == 3){ //Move back

            //Drive
            Pose2d thirdStep = new Pose2d(start.getX() - 1, start.getY(), 0);
            movment.moveToDSB(thirdStep, 0.8);
            double[] distances = distanceNumbers(currentPose, thirdStep);

            //Slides
            double slideToLocation = hight-82;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.3, distances[0], 1.5)) {
                placeState++;
            }
        }


        if(placeState == 4){ //Move back

            //Drive
            Pose2d thirdStep = new Pose2d(start.getX() - 5, start.getY(), 0);
            movment.moveToDSB(thirdStep, 0.8);
            double[] distances = distanceNumbers(currentPose, thirdStep);

            //Slides
            double slideToLocation = hight-73;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.3, distances[0], 5)) {
                placeState++;
            }
        }

        if(placeState == 5){ //finished
            placeState = 0;
            return true;
        }

        return false;
    }

    public boolean pushSpec(Pose2d dep, double h){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (pushState == 0) { // Move to bottom left

            //Drive
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(-1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if (pushState == 1) { // Move to top left

            //Drive
            movment.moveToDSB(TOP_LEFT_CORNER_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, TOP_LEFT_CORNER_SPEC1);

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

        if (pushState == 2) { // Move to top right

            //Drive
            movment.moveToDSB(TOP_MIDDLE_CORNER_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

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

        if (pushState == 3) { // Move to observation zone

            //Drive
            movment.moveTo(OBSERVATION_ZONE_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC1);

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
        //TODO start

        if (pushState == 4) { // Move back up

            //Drive
            movment.moveTo(TOP_MIDDLE_CORNER_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

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
        if (pushState == 5) { // Move to the right

            //Drive
            movment.moveTo(TOP_RIGHT_CORNER_SPEC1, 1);
            double[] distances = distanceNumbers(currentPose, TOP_RIGHT_CORNER_SPEC1);

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

        if (pushState == 6) { // Move to observation zone

            //Drive
            movment.moveTo(OBSERVATION_ZONE_SPEC2, 1);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC2);

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
        //TODO finish

        if (pushState == 7) { // Move Back

            //Drive
            movment.moveTo(IntakeSet, 0.7);
            double[] distances = distanceNumbers(currentPose, IntakeSet);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.2, distances[0], 1)) {
                pushState++;
            }
        }

        if (pushState == 5) { // Intake

            //Drive
            movment.moveTo(INTAKE2, 0.4);
            double[] distances = distanceNumbers(currentPose, INTAKE2);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.2, distances[0], 1)) {
                pushState++;
            }
        }

        if (pushState == 6) { // SetUpforDeposit

            //Drive
            movment.moveTo(DEPOSIT2);
            double[] distances = distanceNumbers(currentPose, DEPOSIT2);

            //Slides
            double slideToLocation = 60;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0.1);

            //Check
            if (checkIfInsideBox(distances[1], .3, distances[0], 4)) {
                pushState++;
            }
        }
        if(pushState == 7){
            boolean placed = plaseSpecimin(dep, h);
            if(placed){
                pushState++;
            }
        }
        if(pushState == 8){

            //Drive
            movment.moveTo(BOTTOM_LEFT_CORNER_SET_UP, 1);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(-0.5);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1.5)) {
                pushState++;

            }
        }
        if(pushState == 9){
            pushState = 0;
            return true;
        }

        return false;
    }

    public boolean preLoad() {
        boolean completed = plaseSpecimin(PRELOAD, 720.5);
        return completed;
    }
    /*
    public boolean setUP(Pose2d intake, Pose2d dep){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (pushState == 0) { // drive

            //Drive
            movment.moveToDSB(TURN, 0.5);
            double[] distances = distanceNumbers(currentPose, TURN);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1)) {
                pushState++;
            }
        }

        if (pushState == 1) { // intaking

            //Drive
            movment.moveToDSB(intake, 0.4);
            double[] distances = distanceNumbers(currentPose, intake);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1)) {
                pushState++;
            }
        }

        if (pushState == 2) { // moveToDeposit

            //Drive
            movment.moveToDSB(dep, 1.1);
            double[] distances = distanceNumbers(currentPose, dep);

            //Slides
            double slideToLocation = 300;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0.1);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 1)) {
                pushState++;
            }
        }

        if(pushState == 3){
            boolean plased = plaseSpecimin(dep);
            if(plased){
                pushState++;
            }
        }
        if(pushState == 4){

            //Drive
            movment.moveToDSB(BOTTOM_LEFT_CORNER_SET_UP, 1.2);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SET_UP);

            //Slides
            double slideToLocation = 0;
            slide.slideTo(slideToLocation);

            //Linkage
            slide.linkageTo(0);

            //Intake
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], .25, distances[0], 5)) {
                pushState++;
            }
        }
        if(pushState == 5){
            return true;
        }

        return false;
    }

     */

    public boolean cycle(Pose2d DEPOSIT, Pose2d INTAKE_CYCLE, double h){
        //Update
        slide.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (cycleState == 0) { // Intake

            //Drive
            movment.moveToDSB(INTAKE_CYCLE, 0.3);
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

        if(cycleState == 1){
            boolean plased = plaseSpecimin(DEPOSIT, h);
            if(plased){
                cycleState++;
            }
        }
        telemetry.addData("substate", cycleState);
        if(cycleState == 2){
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