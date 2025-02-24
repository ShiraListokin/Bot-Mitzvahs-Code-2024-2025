package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;
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

@Autonomous(name="testAuto")
public class AutoTestStates extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private utilMovment util;

    @Override
    public void runOpMode() {
        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //SubSystems
        drive = new SampleMecanumDrive(hardwareMap);
        util = new utilMovment(drive);

        util = new utilMovment(drive);

        //PreSetValues
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            util.moveTo(new Pose2d(0, 10, 0));
            drive.update();
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Theta", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}