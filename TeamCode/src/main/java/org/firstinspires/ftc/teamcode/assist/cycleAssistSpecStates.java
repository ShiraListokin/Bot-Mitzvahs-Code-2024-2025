package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slide;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class cycleAssistSpecStates extends cycleAssistStates{

    private Telemetry telemetry;

    //States
    private int placeState;
    private int moveState;
    private int cycleState;

    //Time
    private long time;

    //Poses
    private final Pose2d SPEC1 = new Pose2d(14, -23.8, 5.8);
    private final Pose2d SPEC1O = new Pose2d(19, -17.8, 4.2);

    private final Pose2d SPEC2 = new Pose2d(14, -33.8, 5.8);
    private final Pose2d SPEC2O = new Pose2d(19, -27.8, 4.2);

    private final Pose2d SPEC3 = new Pose2d(13, -41.8, 5.8);
    private final Pose2d SPEC3O = new Pose2d(10, -17.8, 4.2);



    public cycleAssistSpecStates(intake i, slide s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r, Telemetry t) {
        super(i, s, m, sa, r);
        telemetry = t;
        placeState = 0;
        moveState = 0;
        cycleState = 0;
        time = 0;
    }

    public boolean plaseSpecimin(Pose2d start) {
        //Update
        slides.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();

        if (placeState == 0) { // move to preSet
            double[] distances = distanceNumbers(currentPose, start);

            //Drive
            movment.moveTo(start, 0.4);

            //Slides
            double slideToLocation = 678;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.drop(0.23);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() > 665){
                placeState++;
            }
        }
        if(placeState == 1){
            double[] distances = distanceNumbers(currentPose, start);

            //Drive
            movment.moveTo(start, 0.8);

            //Slides
            double slideToLocation = 678;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.5);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() > 665){
                placeState++;
            }
        }
        if(placeState == 2){
            double[] distances = distanceNumbers(currentPose, new Pose2d(start.getX()+5, start.getY(), start.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(start.getX()+5, start.getY(), start.getHeading()), 0.8);

            //Slides
            double slideToLocation = 678;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.5);

            //intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() > 665){
                placeState++;
            }
        }
        if(placeState == 3){
            double[] distances = distanceNumbers(currentPose, new Pose2d(start.getX()+5, start.getY(), start.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(start.getX()+5, start.getY(), start.getHeading()), 0.8);

            //Slides
            double slideToLocation = 652;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0.5);

            //intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5) && slides.getPos() < 659){
                placeState++;
            }
        }
        if(placeState == 4){
            double[] distances = distanceNumbers(currentPose, start);

            //Drive
            movment.moveTo(start, 0.8);

            //Slides
            double slideToLocation = 653;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 2)){
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



    public boolean cycleSpec(Pose2d plase, Pose2d intake){

        slides.update();
        in.update();
        drive.update();

        Pose2d currentPose = drive.getPoseEstimate();


        if (cycleState == 0) { // moveTo Intake
            double[] distances = distanceNumbers(currentPose, intake);

            //Drive
            movment.moveTo(intake);

            //Slides
            double slideToLocation = 5;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(0);
            in.drop(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5)) {
                cycleState ++;
            }
        }

        if (cycleState == 1) { // intake
            double[] distances = distanceNumbers(currentPose, new Pose2d(intake.getX(), intake.getY()-5, intake.getHeading()));

            //Drive
            movment.moveTo(new Pose2d(intake.getX(), intake.getY()-5, intake.getHeading()), 0.3);

            //Slides
            double slideToLocation = 25;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(.8);

            //intake
            in.direction(1);
            in.drop(1);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1.5)) {
                cycleState ++;
            }
        }
        if (cycleState == 2) { // turn and get ready
            double[] distances = distanceNumbers(currentPose, plase);

            //Drive
            movment.moveToRot(plase, 0.8);

            //Slides
            double slideToLocation = 678;
            slides.slideTo(slideToLocation);

            //extension
            slides.extend(0);

            //intake
            in.direction(0.3);
            in.drop(0.6);

            //Check
            if (checkIfInsideBox(distances[1], .5, distances[0], 1) && slides.getPos() > 600) {
                cycleState ++;
            }
        }
        if(cycleState == 3){
            boolean moveOnQuestion = plaseSpecimin(plase);
            if(moveOnQuestion){
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
        slides.update();
        in.update();
        drive.update();

        movment.moveTo(new Pose2d(6, -20, 0));

        //Slides
        double slideToLocation = 5;
        slides.slideTo(slideToLocation);

        //extension
        slides.extend(0);

        //intake
        in.direction(0);
        in.drop(1);
    }

}
