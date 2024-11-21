package org.firstinspires.ftc.teamcode.opModes.old.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
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
            leftLinkage.setPosition(0.15);
            rightLinkage.setPosition(1);
        }
        if(gamepad1.a){
            leftLinkage.setPosition(.75);
            rightLinkage.setPosition(.4);
        }
    }
}

