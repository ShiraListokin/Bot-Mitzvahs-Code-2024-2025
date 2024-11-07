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
public class ServoTesting extends OpMode {

    Servo rightLinkage;

    Servo leftLinkage;

    @Override
    public void init() {
        //slide = hardwareMap.get(DcMotorEx.class, "slide");
        rightLinkage = hardwareMap.get(Servo.class, "RLinkage");
        leftLinkage = hardwareMap.get(Servo.class, "LLinkage");
    }

    @Override
    public void loop() {
        if(gamepad1.y){
            leftLinkage.setPosition(1);
            rightLinkage.setPosition(0.64);
        }
        if(gamepad1.a){
            leftLinkage.setPosition(1);
            rightLinkage.setPosition(0);
        }
    }
}

