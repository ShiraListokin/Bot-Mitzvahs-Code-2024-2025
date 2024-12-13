package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.assist.cycleAssistYellow;

//SubSystems
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

import java.util.concurrent.TimeUnit;

@Autonomous(name="YellowAuto")
public class YellowAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;

    private utilMovment util;

    private intake in;
    private slides slide;

    private final Pose2d BASKET = new Pose2d(9.5, -13.5, Math.PI/4);

    private final Pose2d PRESET1 = new Pose2d(9.6, -38, 3*Math.PI/2);

    private final Pose2d PRESET2 = new Pose2d(19.6, -38, 3*Math.PI/2);

    private final Pose2d PRESET3 = new Pose2d(22, -32.2, (7*Math.PI/4)+0.25);

    private int state = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        in = new intake(hardwareMap, telemetry);
        slide = new slides(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0,0));
        util = new utilMovment(drive);

        cycleAssistYellow assist = new cycleAssistYellow(in, slide, util, drive, runtime);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("time", runtime.time(TimeUnit.SECONDS));
            telemetry.update();

            if(state == 0){
                boolean moveOn = assist.preLoad();
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 1){
                boolean moveOn = assist.cycle(PRESET1, BASKET, 15, 0, 2500, 0, 0, -1);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 2){
                boolean moveOn = assist.cycle(PRESET2, new Pose2d(9.5, -13, Math.PI/4), 15, 0, 2500, 0, 0, -1);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 3){
                boolean moveOn = assist.cycle(PRESET3, new Pose2d(9, -12.5, Math.PI/4), 5, 3, 3000, 0.5, 6, 1500);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 4){
                slide.slideTo(0);
                slide.update();
            }
        }
    }
}