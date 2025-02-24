package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slide;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
public class MainTeleOpTest extends OpMode {
    private SampleMecanumDrive drive;

    private utilMovmentTeleOp movment;
    private int time;
    private slide slides;
    private intake in;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        in = new intake(hardwareMap, telemetry);
        slides = new slide(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        movment.robotCentricDriver();
        slides.update();
        in.update();

        slides.extend(gamepad1.right_trigger);

        if(gamepad1.dpad_up){
            slides.slideTo(1160);
            in.direction(0);
        }
        if(gamepad1.dpad_down){
            slides.slideTo(80);
            in.direction(0);
        }
        if(gamepad1.dpad_right){
            slides.slideTo(678);
            in.direction(1);
        }
        if(gamepad1.dpad_left){
            slides.slideTo(470);
            in.direction(0);
        }

        if(gamepad1.right_bumper){
            in.direction(1);
            in.bounce(true);
            slides.move(-80);
        }
        else if(gamepad1.left_bumper){
            in.direction(-1);
            in.bounce(false);
            slides.move(0);
        }
        else{
            in.direction(0.1);
            in.bounce(false);
            slides.move(0);
        }


        /*
        DcMotorEx LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        DcMotorEx Rightlide = hardwareMap.get(DcMotorEx.class, "RSlide");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        DcMotorEx hang = hardwareMap.get(DcMotorEx.class, "hang");
        ServoImplEx drop = hardwareMap.get(ServoImplEx.class, "drop");
        System.currentTimeMillis();
        slides.slideTo(20);
        slides.update();
        if(gamepad1.x){
            slides.extend(0);
        }
        if(gamepad1.y){
            slides.extend(1);
        }
        if(gamepad1.a){ //back
            time++;
            if(time < 100){
                drop.setPosition(0.07);
            }
            else{
                drop.setPosition(0);
                if(time > 200){
                    time = 0;
                }
            }
        }
        if(gamepad1.b){ //back
            drop.setPosition(0.2);
        }


        if(gamepad1.dpad_up){
            in.direction(1);
        }
        else if(gamepad1.dpad_down){
            in.direction(-1);
        }
        else{
            in.direction(0);
        }
        in.update();

        /*
        if(gamepad1.x){ //forward
            rightRear.setPower(.2);
        }
        else{
            rightRear.setPower(0);
        }

        if(gamepad1.y){ //forward
            rightFront.setPower(.2);
        }
        else{
            rightFront.setPower(0);
        }
        */


        /*
        if (gamepad1.right_trigger > 0.1) {
            Rightlide.setPower(gamepad1.right_trigger); //forward
            LeftSlide.setPower(gamepad1.right_trigger);
            hang.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            Rightlide.setPower(-gamepad1.left_trigger); //forward
            LeftSlide.setPower(-gamepad1.left_trigger);
            hang.setPower(-gamepad1.left_trigger);
        }
        else{
            Rightlide.setPower(0); //forward
            LeftSlide.setPower(0);
            hang.setPower(0);
        }
        */


        /*
        if(gamepad1.dpad_up){
            slides.slideTo(1120);
        }
        if(gamepad1.dpad_right){
            slides.slideTo(1120);
        }
        if(gamepad1.dpad_left){
            slides.slideTo(0);
        }
        slides.update();




        if(gamepad1.dpad_up){
            in.direction(1);
        }
        else if(gamepad1.dpad_down){
            in.direction(-1);
        }
        else{
            in.direction(0);
        }
        in.update();

        ServoImplEx rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        ServoImplEx leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");
        if(gamepad1.a){
            rightLinkage.setPosition(.1);
        }
        if(gamepad1.b){
            rightLinkage.setPosition(1);
        }
        if(gamepad1.x){
            leftLinkage.setPosition(0);
        }
        if(gamepad1.y){
            leftLinkage.setPosition(1);
        }

         */
    }
}

