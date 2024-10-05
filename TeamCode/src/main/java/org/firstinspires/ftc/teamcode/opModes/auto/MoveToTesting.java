package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;


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

@TeleOp(name="moveToTest", group="Linear OpMode")
@Disabled
public class MoveToTesting extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    SampleMecanumDrive drive;
    utilMovment util;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0,0));
        util = new utilMovment(drive);
        Pose2d idealPose = new Pose2d(12, 12, Math.PI);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double[] rotationSpeed = util.moveTo(idealPose);
            drive.update();
            telemetry.addData("thinkX", drive.getPoseEstimate().getX());
            telemetry.addData("thinkY", drive.getPoseEstimate().getY());
            telemetry.addData("thinkRot", drive.getPoseEstimate().getHeading());
            telemetry.addData("rotSpeed", rotationSpeed[0]);
            telemetry.addData("pSpeed", rotationSpeed[1]);
            telemetry.addData("speed", rotationSpeed[2]);
            telemetry.addData("heading", rotationSpeed[3]);
            telemetry.addData("Ajusted", rotationSpeed[4]);
            telemetry.update();
        }
    }
}
