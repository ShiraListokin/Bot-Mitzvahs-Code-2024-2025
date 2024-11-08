package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intake {
    private CRServoImplEx LIntake, RIntake;

    private Telemetry telemetry;

    boolean intaking;
    public intake(HardwareMap hardwareMap, Telemetry t){
        telemetry = t;
        LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");
    }

    public void direction(boolean i){
        intaking = i;
    }

    public void update(){
        if(intaking){
            RIntake.setPower(1);
            LIntake.setPower(-1);
            telemetry.addData("Direction", "intake");
        }
        else{
            RIntake.setPower(-1);
            LIntake.setPower(1);
            telemetry.addData("Direction", "deposit");
        }
    }

}