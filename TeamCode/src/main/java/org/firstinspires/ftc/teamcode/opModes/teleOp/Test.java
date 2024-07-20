package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;


@TeleOp
@Disabled
public class Test extends OpMode{

    SampleMecanumDrive drive;

    @Override
    public void init() {

        drive = new SampleMecanumDrive(hardwareMap);

    }

    @Override
    public void loop() {

        //game controller input
        double rightStickY = -gamepad1.right_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;

        //imu input;

        //Rotation calculations
        double idealRotation = (Math.atan2(leftStickY, leftStickX));
        double actualAngle = drive.getRawExternalHeading();
        double rotationSpeed = Math.hypot(leftStickY, leftStickX);

    /*    double angleDifference = idealRotation - botHeading2;

        if (angleDifference > Math.PI) {
            angleDifference -= 2 * Math.PI;
        }
        else if (angleDifference < -Math.PI) {
            angleDifference += 2 * Math.PI;
        }
        int sign = (angleDifference > 0) ? 1 : -1;

        //movment calculations
        double heading = (- (botHeading + Math.PI/2) + Math.atan2(rightStickY, rightStickX)) + Math.PI;
        double speed = Math.hypot(rightStickY,rightStickX);

        double angle;
        if(sign == 1){
            //turning left
            angle = normalizeAngle(angleDifference);
            rotationSpeed *= angle;
        }
        if(sign == -1){
            //turning left
            angle = normalizeAngle(angleDifference);
            rotationSpeed *= (angle-Math.PI);
        }


        //heading turn
        heading = heading - (Math.PI/4);

        drive.setMotorPowers(Math.cos(heading)*speed - sign*rotationSpeed, Math.sin(heading)*speed - sign*rotationSpeed,
                Math.cos(heading)*speed + sign*rotationSpeed, Math.sin(heading)*speed + sign*rotationSpeed);
        */
    }


}
