package org.firstinspires.ftc.teamcode.subSystems.current;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class slideStates {

    DcMotorEx LeftSlide, RightSlide, hang;

    private ServoImplEx leftLinkage, rightLinkage;

    private PIDController PID;

    private Telemetry telemetry;

    private double slidePower;

    private double idealPosition;

    private double extend;
    private double m;
    private boolean updateQ;
    private long time;

    public slideStates(HardwareMap hardwareMap, Telemetry t){
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RSlide");
        hang = hardwareMap.get(DcMotorEx.class, "hang");

        rightLinkage = hardwareMap.get(ServoImplEx.class, "RLinkage");
        leftLinkage = hardwareMap.get(ServoImplEx.class, "LLinkage");

        telemetry = t;

        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        PID = new PIDController(0.02, 0, 0);
        extend = 0;
        m = 0;
        updateQ = true;
        time = 0;
    }

    public void update(){
        if(updateQ) {
            double power = PID.calculate((RightSlide.getCurrentPosition() / 384.5) * 38.3 * Math.PI - (idealPosition + /*33.0*extend */+m));

            telemetry.addData("slide position", (RightSlide.getCurrentPosition() / 384.5) * 38.3 * Math.PI);
            telemetry.addData("Slide Position", idealPosition);

            slidePower = 0.3 + power;

            if (slidePower > 1.0) {
                slidePower = 1.0;
            }
            if (slidePower < -1.0) {
                slidePower = -1.0;
            }

            telemetry.addData("slide power", slidePower);
            telemetry.addData("L current draw", LeftSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("R current draw", RightSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("h draw", hang.getCurrent(CurrentUnit.AMPS));


            LeftSlide.setPower(slidePower);
            RightSlide.setPower(slidePower);
            hang.setPower(slidePower);

            //Linkage
        }
    }

    public void slideTo(double i){
        idealPosition = i;
    }

    public void power(double power){
        LeftSlide.setPower(power);
        RightSlide.setPower(power);
        hang.setPower(power);
    }

    public double getPos(){
        return (RightSlide.getCurrentPosition()/384.5)*38.3*Math.PI;
    }

    public void extend(double position){
        if(position > 0.9){
            position = 0.9;
        }
        leftLinkage.setPosition(position);
        rightLinkage.setPosition(1-position);
        extend = position;
    }

    public void move(double amount){
        m = amount;
    }

    public void mZero(){
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoZero(){
        updateQ = false;
        power(-1);
        if((RightSlide.getCurrent(CurrentUnit.AMPS) > 3.5)){
            mZero();
            updateQ = true;
            power(0);
        }
    }
    public double ampage(){
        return RightSlide.getCurrent(CurrentUnit.AMPS);
    }

    public void exit() {
        updateQ = true;
    }

    }
