package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.SampleTankDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadRunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class utilMovment {

    protected SampleMecanumDrive drive;
    protected PIDController translationalPID;

    protected PIDController headingPID;

    public utilMovment(SampleMecanumDrive drive1){
        drive = drive1;
        translationalPID = new PIDController(0.25, 0, 0.02); //tune
        headingPID = new PIDController(.8, 0, 0); //tune

    }

    public double[] moveTo(Pose2d idealPose){
        double[] array = moveTo(idealPose, 1.0);
        return array;
    }

    public double[] moveTo(Pose2d idealPose, double s){
        Pose2d currentPose = drive.getPoseEstimate();

        //Givens (GO MR FEILD)
        double xi = currentPose.getX();
        double yi = currentPose.getY();
        double thetai = currentPose.getHeading();

        double xf = idealPose.getX();
        double yf = idealPose.getY();
        double thetaf = idealPose.getHeading();

        double deltaX = xf-xi;
        double deltaY = yf-yi;

        //speed
        double heading = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaX, deltaY);
        double speed = Math.abs(translationalPID.calculate(distance));

        //speedCap
        if(speed > s){
            speed = s;
        }
        if(speed < 0.15){
            speed = 0.15;
        }

        //angleGivens
        double idealAngle = normalizeAngle(thetaf);
        double currentAngle = normalizeAngle(thetai);

        //directionToTurn
        double sign;
        if (clockwise(idealAngle, currentAngle)){
            sign = 1.0;
        }
        else{
            sign = -1.0;
        }

        //rotSpeed
        double angleInBetween = angleBetween(idealAngle, currentAngle);
        double rotationSpeed = Math.abs(headingPID.calculate(angleInBetween));

        //rotationCap
        if(rotationSpeed > 0.4){
            rotationSpeed = 0.4;
        }

        convertToRobotCentric(speed, heading, currentAngle, sign, rotationSpeed);
        double[] array = {rotationSpeed, speed, heading, heading - currentAngle};
        return(array);
    }

    public double[] moveToDSB(Pose2d idealPose, double sCap){
        Pose2d currentPose = drive.getPoseEstimate();

        //Givens (GO MR FEILD)
        double xi = currentPose.getX();
        double yi = currentPose.getY();
        double thetai = currentPose.getHeading();

        double xf = idealPose.getX();
        double yf = idealPose.getY();
        double thetaf = idealPose.getHeading();

        double deltaX = xf-xi;
        double deltaY = yf-yi;

        //speed
        double heading = Math.atan2(deltaY, deltaX);

        double speed = sCap;
        //angleGivens
        double idealAngle = normalizeAngle(thetaf);
        double currentAngle = normalizeAngle(thetai);

        //directionToTurn
        double sign;
        if (clockwise(idealAngle, currentAngle)){
            sign = 1.0;
        }
        else{
            sign = -1.0;
        }

        //rotSpeed
        double angleInBetween = angleBetween(idealAngle, currentAngle);
        double rotationSpeed = Math.abs(headingPID.calculate(angleInBetween));

        //rotationCap
        if(rotationSpeed > 0.4){
            rotationSpeed = 0.4;
        }

        convertToRobotCentric(speed, heading, currentAngle, sign, rotationSpeed);
        double[] array = {rotationSpeed, speed, heading, heading - currentAngle};
        return(array);
    }

    protected double normalizeAngle(double angle){
        double twoPi = 2 * Math.PI;
        // Use modulus to bring the angle within the range -2pi to 2pi
        angle = angle % twoPi;
        // If the angle is negative, add 2pi to bring it within the range 0 to 2pi
        if (angle < 0) {
            angle += twoPi;
        }
        return angle;
    }
    protected boolean clockwise (double idealAngle, double currentAngle){
        double eval = currentAngle - idealAngle;
        eval = normalizeAngle(eval);
        if(eval > Math.PI){
            return true;
        }
        return false;
    }

    protected static double angleBetween(double angle1, double angle2) {
        // Normalize the angles to be within 0 to 2π
        angle1 = angle1 % (2 * Math.PI);
        angle2 = angle2 % (2 * Math.PI);

        // Calculate the difference
        double diff = Math.abs(angle1 - angle2);

        // Ensure the angle is the smallest possible
        if (diff > Math.PI) {
            diff = (2 * Math.PI) - diff;
        }

        return diff;
    }
    public void convertToRobotCentric (double speed, double feildHeading, double robotDirection, double rotationDirection, double rotationSpeed){
        double heading = feildHeading - robotDirection;
        heading += Math.PI/4;
        double RF = Math.sin(heading)*speed + rotationDirection*rotationSpeed;
        double RB = Math.cos(heading)*speed + rotationDirection*rotationSpeed;
        double LF = Math.cos(heading)*speed - rotationDirection*rotationSpeed;
        double LB = Math.sin(heading)*speed - rotationDirection*rotationSpeed;

        drive.setMotorPowers(LF, LB, RB, RF);
    }

}
