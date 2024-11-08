package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.slides;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;


@TeleOp
public class MainTeleOp extends OpMode {
    SampleMecanumDrive drive;

    utilMovmentTeleOp movment;

    intake in;
    slides slide;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        slide = new slides(hardwareMap, telemetry);
        in = new intake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        slide.update();
        in.update();
        if(gamepad1.dpad_up){
            slide.slideTo(1050); //high basket
        }
        if(gamepad1.dpad_down){
            slide.slideTo(0); //rest (will be higher later)
        }
        if(gamepad1.dpad_left){
            slide.slideTo(400); //Hang
        }
        if(gamepad1.dpad_right){
            slide.slideTo(500); //chamber
        }
        if (gamepad1.y) {
            in.direction(false);
        }
        else{
            in.direction(true);
        }
    }
}

