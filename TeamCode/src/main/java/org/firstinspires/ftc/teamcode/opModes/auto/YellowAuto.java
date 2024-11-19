package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;

import java.util.concurrent.TimeUnit;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="YellowAuto")
public class YellowAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive drive;

    utilMovment util;

    intake in;
    slides slide;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        in = new intake(hardwareMap, telemetry);
        slide = new slides(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0,0));

        util = new utilMovment(drive);
        Pose2d idealPose = new Pose2d(10, -13, Math.PI/4);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            slide.linkageTo(0);
            telemetry.addData("time", runtime.time(TimeUnit.SECONDS));
            telemetry.update();
            drive.update();
            if((runtime.time(TimeUnit.SECONDS) > 1 && runtime.time(TimeUnit.SECONDS) < 9)|| runtime.time(TimeUnit.SECONDS) > 22){
                slide.slideTo(1150);
            }
            else{
                slide.slideTo(0);
            }
            slide.update();
            if((runtime.time(TimeUnit.SECONDS) > 5 && runtime.time(TimeUnit.SECONDS) < 8) || runtime.time(TimeUnit.SECONDS) > 24){
                in.direction(-1);
            }
            else if(runtime.time(TimeUnit.SECONDS) > 8){
                in.direction(1);
            }
            if(runtime.time(TimeUnit.SECONDS) < 11){
                util.moveTo(idealPose);
            }
            else if(runtime.time(TimeUnit.SECONDS) < 14){
                util.moveTo(new Pose2d(9.6, -23, 3*Math.PI/2));
            }
            else if(runtime.time(TimeUnit.SECONDS) < 18){
                util.moveTo(new Pose2d(9.6, -33, 3*Math.PI/2));
            }
            else{
                util.moveTo(idealPose);
            }
            in.update();
        }
    }
}
