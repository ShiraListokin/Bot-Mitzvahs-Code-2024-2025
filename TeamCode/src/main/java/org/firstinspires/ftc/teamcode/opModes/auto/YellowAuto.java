package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.assist.cycleAssistYellow;
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

    private SampleMecanumDrive drive;

    private utilMovment util;

    private intake in;
    private slides slide;

    private final Pose2d BASKET = new Pose2d(10, -13, Math.PI/4);

    private final Pose2d PRESET1 = new Pose2d(9.6, -38, 3*Math.PI/2);

    private final Pose2d PRESET2 = new Pose2d(19.6, -38, 3*Math.PI/2);
    final Pose2d PRESET3 = new Pose2d(21.67, -40, 7*Math.PI/4);

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
                boolean moveOn = assist.cycle(PRESET1, BASKET, 15);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 2){
                boolean moveOn = assist.cycle(PRESET2, new Pose2d(10, -12, Math.PI/4), 15);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
            if(state == 3){
                boolean moveOn = assist.cycle(PRESET3, new Pose2d(10, -11, Math.PI/4), 20);
                if(moveOn){
                    state ++;
                    assist.reset();
                }
            }
        }
    }
}
