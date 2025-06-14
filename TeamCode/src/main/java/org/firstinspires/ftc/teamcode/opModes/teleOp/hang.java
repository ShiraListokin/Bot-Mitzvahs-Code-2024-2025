package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.slides;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovmentTeleOp;


@TeleOp
public class hang extends OpMode {
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
        in.direction(1);
        in.update();
        telemetry.addData("slide pos", slide.getPos());
        telemetry.update();
    }
}

