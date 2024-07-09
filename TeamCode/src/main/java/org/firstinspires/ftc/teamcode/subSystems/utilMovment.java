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

    private SampleMecanumDrive drive;
    private PIDController translationalPID;

    private PIDFController headingPID;

    public utilMovment(SampleMecanumDrive drive1){
        drive = drive1;
        translationalPID = new PIDController(0.4, 0, .015);
        headingPID = new PIDFController(.4, 0, 0,0);

    }

    public double[] moveTo(Pose2d idealPose){
        Pose2d currentPose = drive.getPoseEstimate();
        double heading = atan2(idealPose.getX()-currentPose.getX(), idealPose.getY()-currentPose.getY());
        double speed = Math.abs(translationalPID.calculate(Math.hypot(currentPose.getX() - idealPose.getX(), currentPose.getY() - idealPose.getY())));
        double pSpeed = speed;
        if(speed > 0.5){
            speed = 0.5;
        }
        double idealAngle = normalizeAngle(idealPose.getHeading());
        double currentAngle = normalizeAngle(currentPose.getHeading());
        double sign;
        if (clockwise(idealAngle, currentAngle)){
            sign = 1.0;
        }
        else{
            sign = -1.0;
        }
        double rotationSpeed = Math.abs(headingPID.calculate(angleBetween(idealAngle, currentAngle)));

        heading -= Math.PI/4;
        double RF = /*Math.sin(heading)*speed*/ + sign*rotationSpeed;
        double RB = /*Math.cos(heading)*speed*/ + sign*rotationSpeed;
        double LF = /*Math.cos(heading)*speed*/ - sign*rotationSpeed;
        double LB = /*Math.sin(heading)*speed*/ - sign*rotationSpeed;

        drive.setMotorPowers(LF, LB, RB, RF);
        double[] array = {rotationSpeed, pSpeed, speed, Math.hypot(currentPose.getX() - idealPose.getX(), currentPose.getY() - idealPose.getY())};
        return(array);
    }

    public double normalizeAngle(double angle){
        double twoPi = 2 * Math.PI;
        // Use modulus to bring the angle within the range -2pi to 2pi
        angle = angle % twoPi;
        // If the angle is negative, add 2pi to bring it within the range 0 to 2pi
        if (angle < 0) {
            angle += twoPi;
        }
        return angle;
    }
    public boolean clockwise (double idealAngle, double currentAngle){
        double eval = currentAngle - idealAngle;
        eval = normalizeAngle(eval);
        if(eval > PI){
            return true;
        }
        return false;
    }

    public static double angleBetween(double angle1, double angle2) {
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


}
