package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assist.cycleAssistYellow;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;

public class SpecAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private SampleMecanumDrive drive;

    private utilMovment util;

    private intake in;
    private slides slide;
    private int state = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        in = new intake(hardwareMap, telemetry);
        slide = new slides(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        util = new utilMovment(drive);

        cycleAssistYellow assist = new cycleAssistYellow(in, slide, util, drive, runtime);

        waitForStart();
        runtime.reset();
    }

    }
