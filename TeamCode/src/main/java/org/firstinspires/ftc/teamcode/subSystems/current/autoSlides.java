package org.firstinspires.ftc.teamcode.subSystems.current;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class autoSlides {

    DcMotorEx LeftSlide, RightSlide;

    ServoImplEx leftLinkage, rightLinkage;

    PIDController PID;

    Telemetry telemetry;

    double slidePower;

    double idealPosition;

    double positionEdit;

    double[] state;

    public autoSlides(HardwareMap hardwareMap, Telemetry t){

        //motors
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");

        //linkage
        rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");

        //telemetry
        telemetry = t;

        //encoderReset and reverse motor
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //PID
        PID = new PIDController(0.02, 0, 0);

        //current state
        state = new double[2];
        state[0] = 0.0; //slide
        state[1] = 0.0; //linkage
    }

    public void setPosition(double p){
        positionEdit = p;
    }

    public double findPosition(){
        return (RightSlide.getCurrentPosition()/384.5)*33.0*Math.PI + positionEdit;
    }

    public void update(){
        double power = PID.calculate(findPosition()-(idealPosition));

        telemetry.addData("slide position", findPosition());

        //gravity
        slidePower = 0.3 + power;

        //speed Cap
        if(slidePower > 1.0){
            slidePower = 1.0;
        }
        if(slidePower < -1.0){
            slidePower = -1.0;
        }

        //telemetry
        telemetry.addData("slide power", slidePower);
        telemetry.addData("L current draw", LeftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("R current draw", RightSlide.getCurrent(CurrentUnit.AMPS));

        //setPower
        LeftSlide.setPower(slidePower);
        RightSlide.setPower(slidePower);

        //stateUpdate
        state[0] = ((RightSlide.getCurrentPosition()/384.5)*33*Math.PI);
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void linkageTo(double idealExtensin){

        double L = .96-(0.645*idealExtensin);
        double R = 0+(0.66*idealExtensin);

        leftLinkage.setPosition(L);
        rightLinkage.setPosition(R);

        state[1] = idealExtensin;
    }

    public double[] state(){
        return state;
    }

    public void killServo(){
        leftLinkage.setPwmDisable();
        leftLinkage.getController().close();
        rightLinkage.setPwmDisable();
        rightLinkage.getController().close();
    }
}
