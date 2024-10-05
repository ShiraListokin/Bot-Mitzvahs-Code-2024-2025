package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.*;

import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class slides {

    private DcMotor rightMotor, leftMotor;

    public slides(HardwareMap hardwareMap){
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Reset encoder
    }

    public void moveTo(double idealPosition){
        //position = (rightMotor.getCurrentPosition()/537.6)*93.0773;
        //double power = PID.calculate(position, idealPosition);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
