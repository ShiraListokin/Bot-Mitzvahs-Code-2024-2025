package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class cycleAssistSpecStates extends cycleAssistStates{

    private Telemetry telemetry;
    private long cycleTime;

    //States
    private int placeState;
    private int moveState;
    private int cycleState;
    private int pushState;
    private int setUpState;

    //poses
    private final Pose2d BOTTOM_LEFT_CORNER_SPEC1 = new Pose2d(26, -26, 3*Math.PI/2); //TODO fill in
    private final Pose2d TOP_LEFT_CORNER_SPEC1 = new Pose2d(49, -27, 3*Math.PI/2); //TODO fill in
    private final Pose2d TOP_MIDDLE_CORNER_SPEC1 = new Pose2d(49, -36, 3*Math.PI/2); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC1 = new Pose2d(9, -37, 3*Math.PI/2); //TODO fill in
    private final Pose2d OBSERVATION_ZONE_SPEC2 = new Pose2d(10, -44, 3*Math.PI/2); //TODO fill in
    private final Pose2d TOP_RIGHT_CORNER_SPEC2 = new Pose2d(49, -47, 3*Math.PI/2); //TODO fill in

    //Time
    private long time;

    //Poses
    private final Pose2d SPEC1 = new Pose2d(14, -23.8, 5.8);
    private final Pose2d SPEC1O = new Pose2d(19, -17.8, 4.2);

    private final Pose2d SPEC2 = new Pose2d(14, -33.8, 5.8);
    private final Pose2d SPEC2O = new Pose2d(19, -27.8, 4.2);

    private final Pose2d SPEC3 = new Pose2d(13, -41.8, 5.8);
    private final Pose2d SPEC3O = new Pose2d(10, -17.8, 4.2);

    public cycleAssistSpecStates(intake i, slideStates s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r, Telemetry t) {
        super(i, s, m, sa, r);
        telemetry = t;
        placeState = 0;
        moveState = 0;
        cycleState = -1;
        time = 0;
        cycleTime = 0;
        pushState = -1;
        setUpState = 0;
    }

    public boolean pushSpec(){
        //Update
        slides.update();

        in.drop(0.8);
        in.update();
        drive.update();
        slides.extend(0);


        Pose2d currentPose = drive.getPoseEstimate();

        if(pushState == -1){
            pushState = 0;
            time = System.currentTimeMillis();
        }
        if (pushState == 0) { // Move to bottom left

            //Drive
            movment.moveTo(BOTTOM_LEFT_CORNER_SPEC1);
            double[] distances = distanceNumbers(currentPose, BOTTOM_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //intake
            in.direction(-0.3);

            //Check
            if (checkIfInsideBox(distances[1], 0.6, distances[0], 1) || System.currentTimeMillis() - time > 850) {
                pushState++;
                time = 0;
            }
        }
        if (pushState == 1) { // Move to top left

            //Drive
            movment.moveToDSB(TOP_LEFT_CORNER_SPEC1, 1.4);
            double[] distances = distanceNumbers(currentPose, TOP_LEFT_CORNER_SPEC1);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);
            in.direction(0);

            //Check
            if (checkIfInsideBox(distances[1], 0.3, distances[0], 4)) {
                pushState++;
            }
        }
        if (pushState == 2) { // Move to top right
            //Drive
            movment.moveTo(TOP_MIDDLE_CORNER_SPEC1);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if (pushState == 3) { // Move to observation zone

            //Drive
            movment.moveTo(OBSERVATION_ZONE_SPEC1, 1.4);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC1);

            //Slide
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //Linkage

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }
        if (pushState == 4) { // Move to top middle

            //Drive
            movment.moveToDSB(TOP_MIDDLE_CORNER_SPEC1, 1.4);
            double[] distances = distanceNumbers(currentPose, TOP_MIDDLE_CORNER_SPEC1);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }
        if (pushState == 5) { // Move to top right
            //Drive
            movment.moveTo(TOP_RIGHT_CORNER_SPEC2, 1);
            double[] distances = distanceNumbers(currentPose, TOP_RIGHT_CORNER_SPEC2);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }

        if (pushState == 6) { // Move to observation zone

            //Drive
            movment.moveTo(OBSERVATION_ZONE_SPEC2, 1.4);
            double[] distances = distanceNumbers(currentPose, OBSERVATION_ZONE_SPEC2);

            //Slides
            double slideToLocation = 10;
            slides.slideTo(slideToLocation);

            //Linkage

            //Check
            if (checkIfInsideBox(distances[1], 0.15, distances[0], 4)) {
                pushState++;
            }
        }
        if(pushState == 7){
            return true;
        }
        return false;
    }

    public boolean plaseSpecimin(Pose2d start, double hightZDif, double speed) {
        //Update
        slides.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (placeState == 0) { // move to preSet
            double[] distances = distanceNumbers(currentPose, start);

            //Drive
            movment.moveTo(start, speed);

            //Slides
            double slideToLocation = 683+hightZDif;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.drop(0.2);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 3.3) && slides.getPos() > 675+hightZDif){
                placeState++;
            }
        }
        if(placeState == 1){
            double[] distances = distanceNumbers(currentPose, start);

            //Drive
            movment.moveTo(start, 0.8);

            //Slides
            double slideToLocation = 683+hightZDif;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() > 675+hightZDif){
                placeState++;
            }
        }
        if(placeState == 2){
            double[] distances = distanceNumbers(currentPose, new Pose2d(start.getX()+5, start.getY(), start.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(start.getX()+5, start.getY(), start.getHeading()), 0.8);

            //Slides
            double slideToLocation = 683+hightZDif;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() > 675+hightZDif){
                placeState++;
                time = System.currentTimeMillis();
            }
        }
        if(placeState == 3){
            double[] distances = distanceNumbers(currentPose, new Pose2d(start.getX()+5, start.getY(), start.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(start.getX()+5, start.getY(), start.getHeading()), 0.8);

            //Slides
            double slideToLocation = 642;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //intake
            in.direction(1);

            //Check
            if ((checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() < 665) || System.currentTimeMillis() - time > 500){
                placeState++;
                time = System.currentTimeMillis();
            }
        }
        if(placeState == 4){
            double[] distances = distanceNumbers(currentPose, new Pose2d(start.getX()-3, start.getY(), start.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(start.getX()-3, start.getY(), start.getHeading()), 0.4);

            //Slides
            double slideToLocation = 636;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 4) || System.currentTimeMillis() - time > 500){
                time = 0;
                placeState++;
            }
        }
        if(placeState == 5){
            placeState = 0;
            return true;
        }
        return false;
    }

    public boolean moveSpec() {
        //Update
        slides.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (moveState == 0) { // moveToSpec1
            double[] distances = distanceNumbers(currentPose, SPEC1);
            telemetry.addData("State", "0");

            //Drive
            movment.moveTo(SPEC1, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.drop(0.1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() < 50) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 1) { // extend
            double[] distances = distanceNumbers(currentPose, SPEC1);
            telemetry.addData("State", "1");

            //Drive
            movment.moveTo(SPEC1, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);
            telemetry.addData("Pose", SPEC1);

            //extension
            slides.extend(1);

            //Check
            if (System.currentTimeMillis() - time > 1200) {
                moveState++;
            }
        }
        if (moveState == 2) { // move
            double[] distances = distanceNumbers(currentPose, SPEC1O);
            telemetry.addData("Pose", SPEC1O);
            telemetry.addData("State", "2");
            //Drive
            movment.moveTo(SPEC1O);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5)) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 3) { // deposit
            double[] distances = distanceNumbers(currentPose, SPEC1O);

            //Drive
            movment.moveTo(SPEC1O);
            telemetry.addData("State", "3");

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(1);

            //intake
            in.direction(-1);

            //Check
            if (System.currentTimeMillis() - time > 300) {
                moveState++;
            }
        }





















        if (moveState == 4) { // moveToSpec2
            double[] distances = distanceNumbers(currentPose, SPEC2);
            telemetry.addData("State", "4");

            //Drive
            movment.moveTo(SPEC2, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //intake
            in.drop(0.3);
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .3, distances[0], 1.5)) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 5) { // intake
            double[] distances = distanceNumbers(currentPose, SPEC2);
            telemetry.addData("State", "5");

            //Drive
            movment.moveTo(SPEC2, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(1);

            //Check
            if (System.currentTimeMillis() - time > 1000) {
                moveState++;
            }
        }
        if (moveState == 6) { // move
            double[] distances = distanceNumbers(currentPose, SPEC2O);
            telemetry.addData("State", "6");

            //Drive
            movment.moveTo(SPEC2O);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5)) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 7) { // deposit
            double[] distances = distanceNumbers(currentPose, SPEC2O);
            telemetry.addData("State", "7");

            //Drive
            movment.moveTo(SPEC2O);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.8);

            //intake
            in.direction(-1);

            //Check
            if (System.currentTimeMillis() - time > 300) {
                moveState++;
            }
        }






        if (moveState == 8) { // moveToSpec3
            double[] distances = distanceNumbers(currentPose, SPEC3);
            telemetry.addData("State", "8");

            //Drive
            movment.moveTo(SPEC3, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.6);

            //intake
            in.drop(0.3);
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .3, distances[0], 1.5)) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 9) { // intake
            double[] distances = distanceNumbers(currentPose, SPEC3);
            telemetry.addData("State", "9");

            //Drive
            movment.moveTo(SPEC3, 0.8);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(1);

            //Check
            if (System.currentTimeMillis() - time > 1000) {
                moveState++;
            }
        }
        if (moveState == 10) { // move
            double[] distances = distanceNumbers(currentPose, SPEC3O);
            telemetry.addData("State", "10");

            //Drive
            movment.moveTo(SPEC3O);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.5);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5)) {
                moveState++;
                time = System.currentTimeMillis();
            }
        }
        if (moveState == 11) { // deposit
            double[] distances = distanceNumbers(currentPose, SPEC3O);
            telemetry.addData("State", "11");

            //Drive
            movment.moveTo(SPEC3O);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.8);

            //intake
            in.direction(-1);

            //Check
            if (System.currentTimeMillis() - time > 300) {
                moveState++;
            }
        }
        if(moveState == 12){
            return true;
        }
        return false;
    }



    public boolean cycleSpec(Pose2d plase, Pose2d intake, double intTime){

        slides.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if(cycleState == -1){
            cycleState ++;
            cycleTime = System.currentTimeMillis();
        }
        if (cycleState == 0) { // moveTo Intake
            double[] distances = distanceNumbers(currentPose, intake);

            //Drive
            movment.moveToDSB(intake, 1.1);

            //Slides
            double slideToLocation = 0;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(-0.3);
            in.drop(0.8);

            //Check
            if (checkIfInsideBox(distances[1], .15, distances[0], 1.5) || System.currentTimeMillis() - cycleTime > intTime) {
                cycleState ++;
                cycleTime = System.currentTimeMillis();
            }
        }

        if (cycleState == 1) { // intake
            double[] distances = distanceNumbers(currentPose, new Pose2d(intake.getX(), intake.getY()-8, intake.getHeading()));

                //Drive
            movment.moveTo(new Pose2d(intake.getX(), intake.getY()-8, intake.getHeading()), 0.9);

            //Slides
            double slideToLocation = 5;
            slides.slideTo(slideToLocation);

            //extension
            if(distances[1] < 0.15){
                slides.extend(.35);
            }
            else{
                slides.extend(0);
            }

            //intake
            in.direction(1);
            in.drop(0.8);

            //Check
            if (checkIfInsideBox(distances[1], .3, distances[0], 1.5) || System.currentTimeMillis() - cycleTime > 700) {
                cycleState ++;
            }
        }

        if (cycleState == 2) { // turn and get ready
            double[] distances = distanceNumbers(currentPose, plase);

            //Drive
            movment.moveToRot(plase, 0.8);

            //Slides
            double slideToLocation = 683;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(1);
            in.drop(0.3);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 3) && slides.getPos() > 500) {
                cycleState ++;
            }
        }

        if(cycleState == 3){
            boolean moveOnQuestion = plaseSpecimin(plase, 6, 0.55);
            if(moveOnQuestion){
                cycleState++;
            }
        }

        if(cycleState == 4){
            cycleState = -1;
            return true;
        }
        return false;
    }

    public void park(){
        slides.update();
        in.update();
        drive.update();

        movment.moveTo(new Pose2d(8, -33, 0));

        //Slides
        double slideToLocation = 5;
        slides.slideTo(slideToLocation);

        //extension
        slides.extend(0);

        //intake
        in.direction(0);
        in.drop(0.4);
    }

    public boolean setUp() {

        Pose2d currentPose = drive.getPoseEstimate();


        slides.update();
        in.update();
        drive.update();

        if(setUpState == 0){
            double[] distances = distanceNumbers(currentPose, new Pose2d(10, -21, 3 * Math.PI / 2));

            movment.moveToDSB(new Pose2d(15, -12, 3 * Math.PI / 2), 1.3);

            //Slides
            double slideToLocation = 5;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(0);
            in.drop(0.4);
            if (checkIfInsideBox(distances[1], .5, distances[0], 4.25)) {
                setUpState ++;
            }
        }
        if(setUpState == 1){
            double[] distances = distanceNumbers(currentPose, new Pose2d(-1, -21, 3*Math.PI/2));

            movment.moveTo(new Pose2d(-1, -21, 3*Math.PI/2), 1.2);

            //Slides
            double slideToLocation = 5;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(0);
            in.drop(0.4);
            if (checkIfInsideBox(distances[1], .5, distances[0], 3)) {
                setUpState ++;
            }
        }
        if(setUpState == 2){
            return true;
        }
        return false;
    }
}
