package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;


@TeleOp
public class Test extends OpMode{

    SampleMecanumDrive drive;

    utilMovmentTeleOp movment;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);

    }

    @Override
    public void loop() {
        Pose2d pose = new Pose2d(0, 72, 0);
        double[] rotationSpeed = movment.moveTo(pose);
        drive.update();
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.addData("direction", rotationSpeed[3]);
    }


}
