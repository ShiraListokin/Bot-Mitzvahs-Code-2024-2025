package org.firstinspires.ftc.teamcode.roadRunner.customTuningOpMode;

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

@TeleOp(name="deadWheelOffsetTest", group="Linear OpMode")
@Disabled

public class deadWheelOffset extends LinearOpMode {

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
        Pose2d idealPose = new Pose2d(0, 0, ((Math.PI)));

        waitForStart();
        runtime.reset();
        double bestOffset = 30;
        double PlXSign = 1;
        double PlYSign = 1;
        double PpXSign = 1;
        double PpYSign = 1;
        double PlX = 1.98706547460600254; //old
        double PlY = -5.4002408143126095; //old
        double PpX = 6.2209923361138495; //old
        double PpY = 0.6209989374813726; //old
        double change = 1;
        int counter = 0;

        while (opModeIsActive()) {
            if(counter == 0){
                double OPlX = PlX;
                PlX += change * PlXSign;
                drive.changeLock(hardwareMap, PlX, PlY, PpX, PpY);
                move180();
                sleep(1000);
                if (Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY()) < bestOffset) {
                    bestOffset = Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY());
                }
                else{
                    PlXSign *= -1;
                    PlX = OPlX;
                }
            }
            if(counter == 1){
                double OPlY = PlY;
                PlY += change * PlYSign;
                drive.changeLock(hardwareMap, PlX, PlY, PpX, PpY);
                move180();
                sleep(1000);
                if (Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY()) < bestOffset) {
                    bestOffset = Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY());
                }
                else{
                    PlYSign *= -1;
                    PlY = OPlY;
                }
            }
            if(counter == 2){
                double OPpX = PpX;
                PpX += change * PpXSign;
                drive.changeLock(hardwareMap, PlX, PlY, PpX, PpY);
                move180();
                sleep(1000);
                if (Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY()) < bestOffset) {
                    bestOffset = Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY());
                }
                else{
                    PpXSign *= -1;
                    PpX = OPpX;
                }
            }
            if(counter == 3){
                double OPpY = PpY;
                PpY += change * PpYSign;
                drive.changeLock(hardwareMap, PlX, PlY, PpX, PpY);
                move180();
                sleep(1000);
                if (Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY()) < bestOffset) {
                    bestOffset = Math.abs(drive.getPoseEstimate().getX()) + Math.abs(drive.getPoseEstimate().getY());
                }
                else{
                    PpYSign *= -1;
                    PpY = OPpY;
                }
            }
            counter ++;
            if(counter == 4){
                counter = 0;
                change *= 0.925;
            }
            //if moved 180deg - check if better - if yes --> set new standard --> else, change the numbers back
            //method that cycles through numbers and changes by 1 in a direction and reverses the direction

            telemetry.addData("thinkX", drive.getPoseEstimate().getX());
            telemetry.addData("thinkY", drive.getPoseEstimate().getY());
            telemetry.addData("thinkRot", drive.getPoseEstimate().getHeading());
            telemetry.addData("BestOffset", bestOffset);
            telemetry.addData("changeBy", change);
            telemetry.addData("PlX", PlX);
            telemetry.addData("PlY", PlY);
            telemetry.addData("PpX", PpX);
            telemetry.addData("PpY", PpY);
            telemetry.update();
        }
    }

    public void move180(){
        while (true){
            util.moveTo(new Pose2d(0, 0, ((Math.PI))));
            drive.update();
            if(Math.abs(drive.getPoseEstimate().getHeading() - Math.PI) < 0.1){
                drive.setMotorPowers(0,0,0,0);
                return;
            }
        }
    }
}