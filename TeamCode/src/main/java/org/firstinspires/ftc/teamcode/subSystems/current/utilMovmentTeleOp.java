package org.firstinspires.ftc.teamcode.subSystems.current;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class utilMovmentTeleOp extends utilMovment {

    private Gamepad gamepad1;
    private double sence;

    private Gamepad gamepad2;
    public utilMovmentTeleOp(SampleMecanumDrive drive1, Gamepad gamepadOne, Gamepad gamepadTwo) {
        super(drive1);
        gamepad1 = gamepadOne;
        gamepad2 = gamepadTwo;
        sence = 1.0;
    }
    double idealAngle;

    public void robotCentricDriver(){
        double X_Left_Stick = gamepad1.left_stick_x;
        double Y_Left_Stick = -gamepad1.left_stick_y;
        double Y_Right_Stick = -gamepad1.right_stick_x;
        double X_Right_Stick = -gamepad1.right_stick_y;

        //        leftFront.setPower(v);
        //        leftRear.setPower(v1);
        //        rightRear.setPower(v2);
        //        rightFront.setPower(v3);

        double robot_heading = (Math.atan2(Y_Right_Stick, X_Right_Stick)) + Math.PI/4;
        double speed = Math.hypot(X_Right_Stick, Y_Right_Stick)*1.2*sence;

        double rotation = 0.73*X_Left_Stick*sence;
        drive.setMotorPowers(Math.cos(robot_heading) * speed + rotation, Math.sin(robot_heading) * speed  + rotation, Math.cos(robot_heading) * speed  - rotation, Math.sin(robot_heading) * speed  - rotation);
    }

    public void setSens(double s){
        sence = s;
    }

    public void feildCentricDrive(){
        double X_Left_Stick = gamepad1.left_stick_x;
        double Y_Left_Stick = -gamepad1.left_stick_y;
        double X_Right_Stick = gamepad1.right_stick_x;
        double Y_Right_Stick = -gamepad1.right_stick_y;

        //Rotation
        double curentHeading = drive.getRawExternalHeading() + Math.PI;
        if(Math.hypot(X_Left_Stick, Y_Left_Stick) > 0.1)
            idealAngle = (Math.atan2(Y_Left_Stick, X_Left_Stick));
        double sign;

        if (clockwise(idealAngle, curentHeading)){
            sign = 1.0;
        }
        else{
            sign = -1.0;
        }
        double rotationSpeed = Math.abs(headingPID.calculate(angleBetween(idealAngle, curentHeading)))*Math.hypot(X_Left_Stick, Y_Left_Stick);
        if (rotationSpeed > 0.2){
            rotationSpeed = 0.2;
        }

        //Translatonal
        double heading = Math.atan2(Y_Right_Stick, X_Right_Stick);
        double speed = Math.hypot(X_Right_Stick,Y_Right_Stick);
        convertToRobotCentric(0.35*speed, heading, curentHeading, sign, rotationSpeed * Math.abs((curentHeading-idealAngle) % 2*Math.PI));
    }
}
