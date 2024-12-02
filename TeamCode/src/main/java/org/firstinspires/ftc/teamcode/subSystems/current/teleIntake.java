package org.firstinspires.ftc.teamcode.subSystems.current;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teleIntake {
    private CRServoImplEx LIntake, RIntake;

    private Telemetry telemetry;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private boolean helperIntakeSpec = false;
    private boolean helperDepositSpec = false;

    private int amountPressedIntake = 0;
    private int amountPressedDeposit = 0;

    public teleIntake(HardwareMap hardwareMap, Telemetry t, Gamepad gamepadOne, Gamepad gamepadTwo ){
        //telemetry
        telemetry = t;
        
        //gamepads
        gamepad1 = gamepadOne;
        gamepad2 = gamepadTwo;
                
        //servos
        LIntake = hardwareMap.get(CRServoImplEx.class, "LIntake");
        RIntake = hardwareMap.get(CRServoImplEx.class, "RIntake");
    }
    
    public void update(){

        if(gamepad2.dpad_right != helperDepositSpec){
            helperDepositSpec = gamepad2.dpad_right;
            amountPressedDeposit ++;
            amountPressedDeposit = (amountPressedDeposit / 2.0);
        }
        else if(gamepad2.b != helperIntakeSpec){
            helperIntakeSpec = gamepad2.b;
            amountPressedIntake ++;
            amountPressedIntake = amountPressedIntake % 2;
        }

        if(gamepad1.right_bumper){ //only when pushed
            direction(-1);
        }
        else if(gamepad1.left_bumper){ //only when pushed
            direction(1);
        }
        else if(gamepad2.y){ //only when pushed
            direction(-1);
        }
        else if(gamepad2.a){ //only when pushed
            direction(1);
        }
        else if(amountPressedIntake == 1){ //stays
            direction(1);
        }
        else if(amountPressedDeposit == 1){ //stays
            direction(1);
        }
        else{
            direction(0.0);
        }
    }

    public void direction(double i){
        RIntake.setPower(i);
        LIntake.setPower(-i);
    }

}