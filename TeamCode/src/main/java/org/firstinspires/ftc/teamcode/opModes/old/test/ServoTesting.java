package org.firstinspires.ftc.teamcode.opModes.old.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ServoTesting extends OpMode {

    Servo rightLinkage;

    Servo leftLinkage;
    DcMotorEx LSlide;
    DcMotorEx RSlide;

    @Override
    public void init() {
        //slide = hardwareMap.get(DcMotorEx.class, "slide");
        rightLinkage = hardwareMap.get(Servo.class, "RLinkage");
        leftLinkage = hardwareMap.get(Servo.class, "LLinkage");
        RSlide = hardwareMap.get(DcMotorEx.class, "RSlide");
        LSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        LSlide.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {

        double extension = gamepad1.left_trigger;
        double LLinkageP = .96-0.645*extension;
        double RRlinkage = 0+0.66*extension;

        leftLinkage.setPosition(LLinkageP);
        rightLinkage.setPosition(RRlinkage);

        double virt = gamepad1.right_trigger;
        RSlide.setPower(virt);
        LSlide.setPower(virt);
    }

}

