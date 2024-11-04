package org.firstinspires.ftc.teamcode.opModes.teleOp;

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

    CRServoImplEx rightIntake;
    CRServoImplEx leftIntake;

    Servo rightLinkage;

    double slidePower = 0;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        //slide = hardwareMap.get(DcMotorEx.class, "slide");
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");
        rightIntake = hardwareMap.get(CRServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(CRServoImplEx.class, "leftIntake");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        if(gamepad1.left_trigger < 0.1){
            LeftSlide.setPower(1 * -gamepad1.right_trigger);
            RightSlide.setPower(1 * gamepad1.right_trigger);
        }
        else{
            LeftSlide.setPower( gamepad1.left_trigger);
            RightSlide.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.dpad_up) {
            rightLinkage.setPosition(.5);
        }
        if (gamepad1.dpad_down) {
            rightLinkage.setPosition(1);
        }
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            slidePower = 0;
            LeftSlide.setPower(0);
            RightSlide.setPower(0);
        }
        if (gamepad1.x) {
            rightIntake.setPower(1);
            leftIntake.setPower(-1);
            telemetry.addData("Direction", "forward");
        }
        if (gamepad1.b) {
            rightIntake.setPower(-1);
            leftIntake.setPower(1);
            telemetry.addData("Direction", "backwards");
        }
        if (gamepad1.y) {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
            telemetry.addData("Direction", "off");
        }
    }
}

