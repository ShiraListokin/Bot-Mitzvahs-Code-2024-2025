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

public class slides {

    DcMotorEx LeftSlide, RightSlide;

    ServoImplEx leftLinkage, rightLinkage;

    PIDController PID;

    Telemetry telemetry;

    double slidePower;

    double idealPosition;

    double slideChanger = 0;

    double slideUp = 0;

    double[] state;
    double positionEdit;

    public slides(HardwareMap hardwareMap, Telemetry t){
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");

        telemetry = t;

        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        PID = new PIDController(0.02, 0, 0);

        state = new double[2];
        state[0] = 0.0; //slide
        state[1] = 0.0; //linkage
    }

    public void update(){
        double power = PID.calculate((RightSlide.getCurrentPosition()/384.5)*33.0*Math.PI-(idealPosition+slideChanger) + positionEdit);


        telemetry.addData("slide position", (RightSlide.getCurrentPosition()/384.5)*33*Math.PI);

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

        state[0] = ((RightSlide.getCurrentPosition()/384.5)*33*Math.PI) + positionEdit;
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void linkageTo(double idealExtensin){

        double L = .9-(0.645*idealExtensin);
        double R = 0.07+(0.66*idealExtensin);

        leftLinkage.setPosition(L);
        rightLinkage.setPosition(R);

        state[1] = idealExtensin;

    }
    public void setPosition(double p){
        positionEdit = p;
    }

    public void slideChanger(double amount){
        slideChanger = amount;
    }

    public double[] state(){
        return state;
    }

    public void linkageToEx(double mm){
        double theta = Math.acos((Math.pow(360,2)-Math.pow(mm,2)-Math.pow(176.725776275,2))/((-2)*mm*176.725776275));
        linkageTo((theta/Math.PI));
    }

    public void extend(double percent){
        double hypot = percent*(2+176.725776275);
        linkageToEx(hypot);

    }
    public void power(double power){
        LeftSlide.setPower(power);
        RightSlide.setPower(power);
    }

}
