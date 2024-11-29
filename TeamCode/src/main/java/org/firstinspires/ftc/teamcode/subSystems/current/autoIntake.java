package org.firstinspires.ftc.teamcode.subSystems.current;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class autoIntake {

    private CRServoImplEx LIntake, RIntake;

    private Telemetry telemetry;

    private double intaking;

    public autoIntake(HardwareMap hardwareMap, Telemetry t){
        //telemetry
        telemetry = t;

        //servos
        LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");
    }

    //auto
    public void direction(double i){
        intaking = i;

    }

    public void update(){
        RIntake.setPower(intaking);
        LIntake.setPower(-intaking);

    }

}