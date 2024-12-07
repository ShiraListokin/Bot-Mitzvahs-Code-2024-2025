package org.firstinspires.ftc.teamcode.opModes.old.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;


@TeleOp
public class robotCentric extends OpMode {
    SampleMecanumDrive drive;
    utilMovment movment;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovment(drive);
    }

    @Override
    public void loop() {

    }
}
