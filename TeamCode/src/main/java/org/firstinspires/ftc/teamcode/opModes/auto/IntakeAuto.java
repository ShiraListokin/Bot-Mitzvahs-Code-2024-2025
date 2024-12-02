package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//CycleAssist
import org.firstinspires.ftc.teamcode.assist.cycleAssistSpec;

//SubSystems
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.autoIntake;
import org.firstinspires.ftc.teamcode.subSystems.current.autoSlides;
import org.firstinspires.ftc.teamcode.subSystems.current.teleIntake;
import org.firstinspires.ftc.teamcode.subSystems.current.teleSlides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;

@TeleOp
public class IntakeAuto extends OpMode{

    SampleMecanumDrive drive;

    utilMovmentTeleOp movment;
    autoIntake in;

    //intake in;
    teleSlides slide;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        slide = new teleSlides(hardwareMap, telemetry, gamepad1, gamepad2);
        in = new autoIntake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        slide.update();
        in.update();
        if(gamepad1.dpad_up){
            slide.slideTo(290);
            in.direction(1);
            slide.linkageTo(0.1);
        }
        if(gamepad1.dpad_down){
            slide.slideTo(400);
        }
    }
}
