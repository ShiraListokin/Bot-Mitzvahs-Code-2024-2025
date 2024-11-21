package org.firstinspires.ftc.teamcode.opModes.old.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.old.objectDetectionUtill;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;


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

@Autonomous(name="ImageRecognition", group="Linear OpMode")
@Disabled
public class ImageRecognition extends LinearOpMode{

    private SampleMecanumDrive drive;
    private utilMovment util;
    private objectDetectionUtill objUtill;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        objUtill = new objectDetectionUtill(drive, telemetry);
        waitForStart();
        while(opModeIsActive()){
            objUtill.telemetryUpdate();
        }
    }
}
