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
            slide.slideTo(1125); //high basket
            slide.linkageTo(0);
        }
        if(gamepad1.dpad_down){
            slide.slideTo(0); //rest (will be higher later)
            slide.linkageTo(0);
        }
        if(gamepad1.dpad_left){
            slide.slideTo(330); //Hang
        }
        if(gamepad1.dpad_right){
            slide.slideTo(650); //chamber
            slide.linkageTo(0.3);
        }
        if (gamepad1.y) {
            in.direction(-1);
        }
        if(gamepad1.a){
            in.direction(1);
        }
        if(gamepad1.b){
            in.direction(0);
        }
        /*if (gamepad1.right_bumper){
            slide.slideChanger(40);
        }
        else{
            slide.slideChanger(0);
        }

         */
        if (gamepad1.left_bumper){
            slide.slideChanger(-4000);
        }
        else{
            slide.slideChanger(0);
        }
        if(gamepad1.x){
            slide.linkageTo(1);
        }
    }
}

