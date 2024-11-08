package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class slides {

    DcMotorEx LeftSlide, RightSlide;

    PIDController PID;

    Telemetry telemetry;

    double slidePower;

    double idealPosition;

    public slides(HardwareMap hardwareMap, Telemetry t){
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        telemetry = t;

        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        PID = new PIDController(0.02, 0, 0);
    }

    public void update(){
        double power = PID.calculate((RightSlide.getCurrentPosition()/384.5)*33*Math.PI-idealPosition);

        telemetry.addData("slide position", (RightSlide.getCurrentPosition()/384.5)*33*Math.PI);

        slidePower = 0.4 + power;

        telemetry.addData("slide power", slidePower);

        LeftSlide.setPower(slidePower);
        RightSlide.setPower(slidePower);
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void linkageTo(double idealExtensin){

    }
}
