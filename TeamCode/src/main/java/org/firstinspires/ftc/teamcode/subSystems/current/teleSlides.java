package org.firstinspires.ftc.teamcode.subSystems.current;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

public class teleSlides {

    //Gamepads
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    //motors
    private DcMotorEx LeftSlide, RightSlide;

    //linkages
    private ServoImplEx leftLinkage, rightLinkage;

    //PID
    private PIDController PID;

    //telemetry
    private Telemetry telemetry;

    private double slidePower;

    //position varriable
    private double idealPosition; //ideal positions
    private double slideChanger; //changes the position baced on bumper
    private double positionEdit; //change the zero position

    private double[] state; //state[0] = slidePosition. state[1] = linkagePositiion

    //time for hang
    private ElapsedTime e;
    private double time;


    public teleSlides(HardwareMap hardwareMap, Telemetry t, Gamepad gamepadOne, Gamepad gamepadTwo) {

        //time
        e = new ElapsedTime();
        time = -1;

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

        idealPosition = 0; //set starting position to 0

        //Gamepads
        gamepad1 = gamepadOne;
        gamepad2 = gamepadTwo;
    }

    public boolean hang(boolean a) { //TODO change
        if (gamepad2.left_bumper) { //hang
            if (time == -1) {
                e.reset();
                time = 0;
            }
            time = e.time(TimeUnit.MILLISECONDS);
            if (time < 10000) {
                setPower(-1);
                return false; //TODO change
            } else if (time < 20000) {
                double percent = (20000.0 - time) / 20000.0;
                double power = percent * -1.0;
                setPower(power);
            }
        }
        return a;
    }


    public void update(){

        if(gamepad2.dpad_up){
            setPosition(1150);//TODO Tune the deposit Position
            linkageTo(0);
        }
        if(gamepad2.dpad_down){
            setPosition(65);//TODO Tune the rest Position
            linkageTo(0);
        }
        if(gamepad2.dpad_right){
            setPosition(650);//TODO Tune the rest Position
            linkageTo(0.3);
        }
        if(gamepad2.dpad_left){
            setPosition(330);//TODO Tune the rest Position
            linkageTo(0);
        }
        /*
        if(gamepad2.left_bumper){ //hang
            if(time == -1){
                e.reset();
                time = 0;
            }
            time = e.time(TimeUnit.MILLISECONDS);
            if(time < 10000){
                setPower(-1);
                return;
            }
            else if(time < 20000){
                double percent = (20000.0 - time)/20000.0;
                double power = percent*-1.0;
                setPower(power);
            }
        }
        */

        double power = PID.calculate(((RightSlide.getCurrentPosition()/384.5)*33.0*Math.PI)-(idealPosition+slideChanger) + positionEdit);


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

    public void setPower(double slidePower){
        LeftSlide.setPower(slidePower);
        RightSlide.setPower(slidePower);
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void linkageTo(double idealExtensin){

        double L = .96-0.645*idealExtensin;
        double R = 0+0.66*idealExtensin;

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

    public void killServo(){
        leftLinkage.setPwmDisable();
        leftLinkage.getController().close();
        rightLinkage.setPwmDisable();
        rightLinkage.getController().close();
    }
}
