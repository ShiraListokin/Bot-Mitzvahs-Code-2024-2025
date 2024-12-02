package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.teleIntake;
import org.firstinspires.ftc.teamcode.subSystems.current.teleSlides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
public class MainTeleOp extends OpMode {
    SampleMecanumDrive drive;

    utilMovmentTeleOp movment;

    teleIntake in;
    teleSlides slide;


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        slide = new teleSlides(hardwareMap, telemetry, gamepad1, gamepad2);
        in = new teleIntake(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        slide.update();
        in.update();
    }
}

