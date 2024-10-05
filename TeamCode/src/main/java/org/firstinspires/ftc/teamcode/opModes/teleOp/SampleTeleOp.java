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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.utilMovmentTeleOp;


@TeleOp
public class SampleTeleOp extends OpMode{

    //SampleMecanumDrive drive;

    //DcMotorEx slide;
    //utilMovmentTeleOp movment;

    CRServoImplEx sideRoller1;
    //CRServoImpl a;
    CRServoImplEx sideRoller2;
    DcMotorEx rI;
    DcMotorEx lI;

    @Override
    public void init() {
        //drive = new SampleMecanumDrive(hardwareMap);
        //movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        //slide = hardwareMap.get(DcMotorEx.class, "slide");
        rI = hardwareMap.get(DcMotorEx.class, "rI");
        lI = hardwareMap.get(DcMotorEx.class, "lI");
        sideRoller1 = hardwareMap.get(CRServoImplEx.class, "sideRoller1");
        sideRoller2= hardwareMap.get(CRServoImplEx.class, "sideRoller2");
    }

    @Override
    public void loop() {
        //movment.feildCentricDrive();
        if (gamepad1.x) {
            /*topRoller.setPower(1);
            counterRoller.setPower(-1);
            sideRoller1.setPower(-1);
            sideRoller2.setPower(1);
            telemetry.addData("Durection", "forwards");

             */
            telemetry.addData("Durection", "forwards");
            rI.setPower(1);
            lI.setPower(-1);
            sideRoller1.setPower(-1);
            sideRoller2.setPower(1);
        }
        if (gamepad1.b) {
            /*
            topRoller.setPower(-1);
            counterRoller.setPower(1);
            sideRoller1.setPower(1);
            sideRoller2.setPower(-1);
            telemetry.addData("Durection", "Backwards");

             */
            telemetry.addData("Durection", "Backwards");
            sideRoller1.setPower(1);
            sideRoller2.setPower(-1);
            rI.setPower(-1);
            lI.setPower(1);

        }
        if (!(gamepad1.b || gamepad1.x)){
            /*
            topRoller.setPower(-1);
            counterRoller.setPower(1);
            sideRoller1.setPower(-1);
            sideRoller2.setPower(1);
            telemetry.addData("Durection", "None");

             */
            telemetry.addData("Durection", "None");
            sideRoller1.setPower(0);
            sideRoller2.setPower(0);
            rI.setPower(0);
            lI.setPower(0);

        }



        /*if(gamepad1.y){
            slide.setPower(-0.4);
            telemetry.addData ("direction", "up");
        }
        if(gamepad1.a){
            slide.setPower(0.4 );
            telemetry.addData ("direction", "down");
        }
        if(gamepad1.dpad_down){
            slide.setPower(0);
        }
        */
        /*if(gamepad1.dpad_up){
            topRoller.setPower(0);
            //counterRoller.setPower(0);
        }*/
        telemetry.update();
    }
}
