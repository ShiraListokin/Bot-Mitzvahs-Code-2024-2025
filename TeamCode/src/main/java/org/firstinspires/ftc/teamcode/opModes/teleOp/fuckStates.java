package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
public class fuckStates extends OpMode {
    private SampleMecanumDrive drive;

    private DcMotor hang;

    private utilMovmentTeleOp movment;

    private boolean out;

    private intake in;
    private slideStates slide;
    private ElapsedTime timer;
    private long time;

    private boolean down;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        movment = new utilMovmentTeleOp(drive, gamepad1, gamepad2);
        slide = new slideStates(hardwareMap, telemetry);
        in = new intake(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        ServoImplEx rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        ServoImplEx leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");
        CRServoImplEx LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        CRServoImplEx RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");

        if(gamepad1.a){
            rightLinkage.setPosition(1);
        }
        else{
            rightLinkage.setPosition(0);
        }
        if(gamepad1.b){
            leftLinkage.setPosition(1);
        }
        else{
            leftLinkage.setPosition(0);
        }
        if(gamepad1.x){
            LIntake.setPower(1);
        }
        else{
            LIntake.setPower(0);
        }
        if(gamepad1.y){
            RIntake.setPower(1);
        }
        else{
            RIntake.setPower(0);
        }

    }
}

