package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//CycleAssist
import org.firstinspires.ftc.teamcode.assist.cycleAssistSpec;

//SubSystems
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

@Autonomous(name="SpecAuto")
public class SpecAuto extends LinearOpMode{

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
        cycleAssistSpec assist = new cycleAssistSpec(in, slide, util, drive, runtime);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(state == 0){
                boolean moveOn = assist.preLoad();
                if(moveOn){
                    state ++;
                }
            }
            if(state == 1){
                boolean moveOn = assist.pushSpec();
                if(moveOn){
                    state ++;
                }
            }
            if(state == 2){
                boolean moveOn = assist.setUP();
                if(moveOn){
                    state ++;
                }
            }
            if(state == 3){
                boolean moveOn = assist.cycle();
                if(moveOn){
                    state ++;
                }
            }
            if(state == 4){
                assist.park();
            }
        }
    }
}
