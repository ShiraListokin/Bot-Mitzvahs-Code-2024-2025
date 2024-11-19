package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovment;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;


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
        movment.convertToRobotCentric(0, 0, 0, -1, .2);
    }
}
