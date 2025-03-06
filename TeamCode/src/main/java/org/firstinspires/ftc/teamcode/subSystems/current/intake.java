package org.firstinspires.ftc.teamcode.subSystems.current;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intake {
    private CRServoImplEx LIntake, RIntake;
    private ServoImplEx drop;

    private Telemetry telemetry;
    private long time;
    private int pos;
    private boolean bo;

    double intaking;
    public intake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;
        LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");
        drop = hardwareMap.get(ServoImplEx.class, "drop");
        time = 0;
    }

    public void direction(double i){
        intaking = i;
    }

    public void update(){
        RIntake.setPower(intaking);
        LIntake.setPower(-intaking);
        if(bo){
            if(pos == 0 && System.currentTimeMillis() - time > 100){
                pos = 1;
                time = System.currentTimeMillis();
                drop(0);
            }
            if(pos == 1 && System.currentTimeMillis() - time > 100){
                pos = 0;
                time = System.currentTimeMillis();
                drop(0.3);
            }
        }

    }

    public void drop (double h){
        drop.setPosition(0.2*h);
        telemetry.addData("dropHight", h);
    }

    public void bounce(boolean b){
        bo = b;
    }


}