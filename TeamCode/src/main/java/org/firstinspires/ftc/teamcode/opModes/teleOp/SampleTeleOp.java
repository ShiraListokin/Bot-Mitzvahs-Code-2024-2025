package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;


@TeleOp
public class SampleTeleOp extends OpMode {
    SampleMecanumDrive drive;

    utilMovmentTeleOp movment;
    DcMotorEx LeftSlide;
    DcMotorEx RightSlide;

    Servo RLinkage;
    Servo LLinkage;
    CRServoImplEx LIntake; // left intake
    CRServoImplEx RIntake; // right intake
    PIDController slide;

    double idealLocation = 0;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);

        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        RLinkage = hardwareMap.get(Servo.class, "RLinkage");
        LLinkage = hardwareMap.get(Servo.class, "LLinkage");

        LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");

        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide = new PIDController(0.02, 0, 0); //Increace p
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        telemetry.addData("slide position", (RightSlide.getCurrentPosition()/384.5)*33*Math.PI);
        if(gamepad1.dpad_up){
            idealLocation = 1100; //high basket
        }
        if(gamepad1.dpad_down){
            idealLocation = 0; //rest (will be higher later)
        }
        if(gamepad1.dpad_right){
            idealLocation = 400; //Hang
        }
        if(gamepad1.dpad_right){
            idealLocation = 400; //chamber
        }
        double PID = slide.calculate((RightSlide.getCurrentPosition()/384.5)*33*Math.PI-idealLocation);
        if(PID>0.6){
            PID = 0.6;
        }
        double slidePower = 0.4+PID;
        telemetry.addData("slide power", slidePower);
        LeftSlide.setPower( -slidePower);
        RightSlide.setPower(slidePower);

        /*if (gamepad1.x) {
            RLinkage.setPosition(0.64);
            //LLinkage.setPosition(0);
        }
        if (gamepad1.a) {
            RLinkage.setPosition(0);
            //LLinkage.setPosition(1);
        }

         */
        if (gamepad1.y) {
            RIntake.setPower(-1);
            LIntake.setPower(1);
            telemetry.addData("Direction", "intake");
        }
        else{
            RIntake.setPower(1);
            LIntake.setPower(-1);
            telemetry.addData("Direction", "deposit");
        }

    }
}

