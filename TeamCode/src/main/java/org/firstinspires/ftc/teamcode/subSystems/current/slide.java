package org.firstinspires.ftc.teamcode.subSystems.current;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class slide {

    DcMotorEx LeftSlide, RightSlide;

    ServoImplEx leftLinkage, rightLinkage;

    PIDController PID;

    Telemetry telemetry;

    double slidePower;

    double idealPosition;

    double extend;
    double m;

    public slide(HardwareMap hardwareMap, Telemetry t){
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");

        telemetry = t;

        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        PID = new PIDController(0.02, 0, 0);
        extend = 0;
        m = 0;
    }

    public void update(){
        double power = PID.calculate((RightSlide.getCurrentPosition()/384.5)*38.3*Math.PI-(idealPosition + /*33.0*extend */+ m));

        telemetry.addData("slide position", (RightSlide.getCurrentPosition()/384.5)*38.3*Math.PI);

        slidePower = 0.3 + power;

        if(slidePower > 1.0){
            slidePower = 1.0;
        }
        if(slidePower < -1.0){
            slidePower = -1.0;
        }

        telemetry.addData("slide power", slidePower);
        telemetry.addData("L current draw", LeftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("R current draw", RightSlide.getCurrent(CurrentUnit.AMPS));

        LeftSlide.setPower(slidePower);
        RightSlide.setPower(slidePower);

        //Linkage
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void power(double power){
        LeftSlide.setPower(power);
        RightSlide.setPower(power);
    }

    public double getPos(){
        return (RightSlide.getCurrentPosition()/384.5)*38.3*Math.PI;
    }

    public void extend(double position){
        leftLinkage.setPosition(1-position);
        rightLinkage.setPosition(position);
        extend = position;
    }

    public void move(double amount){
        m = amount;
    }

}
