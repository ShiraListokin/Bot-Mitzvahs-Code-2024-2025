package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//CycleAssist

//SubSystems
import org.firstinspires.ftc.teamcode.assist.cycleAssistSpecStates;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

@Autonomous(name="SpecAutoStates")
public class SpecAutoStates extends LinearOpMode{

    //SubSystems
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private utilMovment util;
    private intake in;
    private slideStates slides;

    //State
    private int state = 0;

    @Override
    public void runOpMode() {
        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //SubSystems
        in = new intake(hardwareMap, telemetry);
        slides = new slideStates(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        util = new utilMovment(drive);

        //PreSetValues
        drive.setPoseEstimate(new Pose2d(-0.5, 0, 0));

        //util
        cycleAssistSpecStates assist = new cycleAssistSpecStates(in, slides, util, drive, telemetry);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Theta", drive.getPoseEstimate().getHeading());
            telemetry.update();

            if (state == 0) {
                boolean moveOn = assist.plaseSpecimin(new Pose2d(21, 1, 0), 0, 0.4);
                if (moveOn) {
                    state++;
                }
            }
            if(state == 2){
                boolean moveOn = assist.pushSpec();
                if (moveOn) {
                    state++;
                }
            }
            if(state == 1){
                boolean moveOn = assist.cycleSpec(new Pose2d(21, 4, 0), new Pose2d(-1.3, -21, 3*Math.PI/2), 2500);
                if (moveOn) {
                    state++;
                }
            }
            if(state == 3){
                boolean moveOn = assist.setUp();
                if (moveOn) {
                    state++;
                }
            }
            if(state == 4){
                boolean moveOn = assist.cycleSpec(new Pose2d(21, 7, 0), new Pose2d(-1, -21, 3*Math.PI/2), 400);
                if (moveOn) {
                    state++;
                }
            }
            if(state == 5){
                boolean moveOn = assist.cycleSpec(new Pose2d(21, 10, 0), new Pose2d(-1, -21, 3*Math.PI/2), 2100);
                if (moveOn) {
                    state++;
                }
            }
            if(state == 6){
               assist.park();
            }
        }
    }

}