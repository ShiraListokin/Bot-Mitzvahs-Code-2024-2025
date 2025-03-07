package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
@Disabled
public class MainTeleOp extends OpMode {
    private SampleMecanumDrive drive;

    private utilMovmentTeleOp movment;

    private boolean out;
    private boolean hang;

    private intake in;
    private slides slide;
    private ElapsedTime timer;
    private long time;

    private boolean down;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        slide = new slides(hardwareMap, telemetry);
        in = new intake(hardwareMap, telemetry);
        out = false;
        timer = new ElapsedTime();
        hang = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Time", time);
        if(gamepad1.a){
            hang = false;
        }
        if(hang == true){
            if (timer.time() < 10000){
                slide.power(-1);
            }
            else if(timer.time() < 20000){
                double power = -1+((timer.time()-10000)/10000);
                slide.power(power);
            }
            else{
                slide.power(0);
            }
        }
        if(hang == false) {

            //Update
            if(gamepad1.b){
                double turn = gamepad1.left_stick_x*0.4;
                drive.setMotorPowers(0.28 + turn, 0.28 + turn, 0.28 - turn, 0.28 - turn);
                slide.slideTo(0);
                slide.linkageTo(0);
                in.direction(1);
            }
            else{
                movment.robotCentricDriver();
            }
            slide.update();
            in.update();

            //intake
            if (gamepad1.right_bumper) {
                in.direction(1);
                movment.setSens(0.5);
                if (out == true) {
                    slide.slideChanger(-60);
                } else {
                    slide.slideChanger(0);
                }
            } else if (gamepad1.left_bumper) {
                in.direction(-1);
                if (out == true) {
                    slide.slideChanger(-60);
                } else {
                    slide.slideChanger(0);
                }
            }
            else {
                in.direction(1);
                slide.slideChanger(0);
                movment.setSens(1.0);
            }


            //extend
            if (gamepad1.right_trigger > 0.5) {
                slide.linkageTo(1);
                slide.slideTo(240);
                out = true;
                movment.setSens(0.6);
                down = false;
            }
            if (gamepad1.left_trigger > 0.5) {
                slide.linkageTo(0);
                slide.slideTo(200);
                out = false;
                movment.setSens(1);
                down = false;
            }

            if (gamepad1.dpad_up) {
                slide.slideTo(1125); //high basket
                slide.linkageTo(0);
                slide.slideChanger(0);
                out = false;
                movment.setSens(0.6);
                down = false;
            }
            if (gamepad1.dpad_down) {
                slide.slideTo(0); //rest
                slide.linkageTo(0);
                slide.slideChanger(0);
                out = false;
                movment.setSens(1);
                time = System.currentTimeMillis();
                down = true;
            }
            if (gamepad1.dpad_left) {
                slide.slideTo(330); //Hang
                slide.linkageTo(0);
                slide.slideChanger(0);
                out = false;
                movment.setSens(1);
                down = false;
            }
            if (gamepad1.dpad_right) {
                slide.slideTo(705); //chamber is at 650
                slide.linkageTo(0.3);
                slide.slideChanger(0);
                out = true;
                movment.setSens(0.8);
                down = false;
            }

            if (gamepad1.y) {
                timer.reset();
                hang = true;
            }
        }

        if(down && gamepad1.x){
            slide.power(-0.7);
            slide.reset();
        }
    }
}

