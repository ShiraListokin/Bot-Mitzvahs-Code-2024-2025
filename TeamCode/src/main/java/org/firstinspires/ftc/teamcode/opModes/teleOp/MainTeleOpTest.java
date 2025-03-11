package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.assist.cycleAssistSpecStates;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
public class MainTeleOpTest extends OpMode {
    private SampleMecanumDrive drive;

    private utilMovmentTeleOp movment;
    private int time;
    private slideStates slides;
    private intake in;
    private boolean notPressed;
    private boolean manual;
    private cycleAssistSpecStates assist;
    private boolean override;
    private boolean pressed;
    private int counter;
    private boolean specMode;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        in = new intake(hardwareMap, telemetry);
        slides = new slideStates(hardwareMap, telemetry);
        assist = new cycleAssistSpecStates(in, slides, movment, drive, telemetry);
        notPressed = true;
        counter = 0;
        specMode = false;
        manual = true;
    }

    @Override
    public void loop() {

        //Update
        movment.robotCentricDriver();
        in.update();

        if (manual) {

            if (gamepad1.right_trigger > 0.1) {
                slides.power(gamepad1.right_trigger);
            } else {
                slides.power(-gamepad1.left_trigger);
            }
            telemetry.addData("slide pos", slides.getPos());
            if (gamepad1.right_bumper) {
                in.direction(1);
                in.bounce(true);
            } else if (gamepad1.left_bumper) {
                in.direction(-1);
                in.bounce(false);
                in.drop(0.8);
            } else {
                in.direction(0.1);
                in.bounce(false);
                in.drop(0.8);
            }
            if (gamepad1.y) {
                slides.mZero();
            }
            if (gamepad1.b) {
                slides.extend(1);
            }
            if (gamepad1.x) {
                slides.extend(0);
            }
            if (gamepad1.a && pressed) {
                manual = false;
                pressed = false;
            }
        }

        if (!manual) {
            slides.update();
            if (gamepad1.a && pressed) {
                manual = true;
                pressed = false;
            }


            //Extend
            if(!specMode){
                slides.extend(gamepad1.right_trigger);
            }

            if (gamepad1.dpad_up) { //basket
                slides.slideTo(1130);
                in.direction(0);
                specMode = false;
            }
            if (gamepad1.dpad_down) { //rest
                slides.slideTo(100);
                in.direction(0);
                specMode = false;
            }
            if (gamepad1.dpad_right) { //Spec Intake
                slides.slideTo(0);
                in.direction(1);
                in.drop(0.8);
                slides.extend(0.3);
                override = true;
                specMode = false;
            } else {
                override = false;
            }

            if (gamepad1.dpad_left && notPressed) {
                specMode = true;
                if(counter == 0){
                    notPressed = false;
                    slides.slideTo(700);
                    slides.extend(0.6);
                    counter = 1;
                    in.direction(1);
                    in.drop(0.2);
                }
                else{
                    notPressed = false;
                    slides.slideTo(663);
                    slides.extend(0);
                    counter = 0;
                    in.direction(1);
                    in.drop(0.2);
                }
            }

            if(!specMode) {
                if (gamepad1.right_bumper) {
                    in.direction(1);
                    in.bounce(true);
                    slides.move(-100);
                } else if (gamepad1.left_bumper) {
                    in.direction(-1);
                    in.bounce(false);
                    slides.move(0);
                    in.drop(0.8);
                } else if (!override) {
                    in.direction(0.1);
                    in.bounce(false);
                    slides.move(0);
                    in.drop(0.8);
                }
            }

            if (gamepad1.y) {
                slides.autoZero();
            } else {
                slides.exit();
            }
        }
        telemetry.update();
        if (!gamepad1.a) {
            pressed = true;
        }
        if (!gamepad1.dpad_left) {
            notPressed = true;
        }
    }
}

