package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//CycleAssist
import org.firstinspires.ftc.teamcode.assist.cycleAssistSpec;

//SubSystems
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

@Autonomous(name="SpecAuto")
public class SpecAuto extends LinearOpMode{

    //Poses
    private final Pose2d DEPOSIT_CYCLE1 = new Pose2d(18.75, 3, 0); //TODO fill in
    private final Pose2d DEPOSIT_CYCLE2 = new Pose2d(19.5, 6.5, 0); //TODO fill in
    private final Pose2d DEPOSIT_CYCLE3 = new Pose2d(20.5, 9, 0); //TODO fill in


    private final Pose2d INTAKE_CYCLE1 = new Pose2d(4, -48, -Math.PI/2); //TODO fill in
    private final Pose2d INTAKE_CYCLE2 = new Pose2d(5, -38, -Math.PI/2); //TODO fill in

    //SubSystems
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private utilMovment util;
    private intake in;
    private slides slide;

    //State
    private int state = 0;

    @Override
    public void runOpMode() {
        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //SubSystems
        in = new intake(hardwareMap, telemetry);
        slide = new slides(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        util = new utilMovment(drive);

        //PreSetValues
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        slide.setPosition(17);

        //util
        cycleAssistSpec assist = new cycleAssistSpec(in, slide, util, drive, runtime, telemetry);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Theta", drive.getPoseEstimate().getHeading());
            telemetry.update();

            if(state == 0){
                boolean moveOn = assist.preLoad();
                if(moveOn){
                    state ++;
                }
            }
            if(state == 1){
                boolean moveOn = assist.pushSpec(DEPOSIT_CYCLE1);
                if(moveOn){
                    state ++;
                }
            }
            if(state == 2){
                boolean moveOn = assist.cycle(DEPOSIT_CYCLE2, INTAKE_CYCLE1);
                if(moveOn){
                    state ++;
                }
            }
            /*
            if(state == 3){
                boolean moveOn = assist.cycle(DEPOSIT_CYCLE2, INTAKE_CYCLE1);
                if(moveOn){
                    state ++;
                    assist.resetCycle();
                }
                }
             */
            /*if(state == 4){
                boolean moveOn = assist.cycle(DEPOSIT_CYCLE3, INTAKE_CYCLE2);
                if(moveOn){
                    state ++;
                    assist.resetCycle();
                }
            }

             */
            /*
            if(state == 5){
                boolean moveOn = assist.cycle(DEPOSIT_CYCLE3, INTAKE_CYCLE2);
                if(moveOn){
                    state ++;
                    assist.resetCycle();
                }
            }
             */
            telemetry.addData("state", state);
            if(state == 3){
                assist.park();
            }
        }
    }
}