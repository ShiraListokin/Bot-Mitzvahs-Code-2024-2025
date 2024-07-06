package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.*;

import androidx.annotation.NonNull;
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
    private PIDFController translationalPID;
    private PIDFController headingPID;

    public utilMovment(SampleMecanumDrive drive1){
        drive = drive1;
        translationalPID = new PIDFController(drive.TRANSLATIONAL_PID.kP,drive.TRANSLATIONAL_PID.kI,drive.TRANSLATIONAL_PID.kD,0);
        headingPID = new PIDFController(drive.HEADING_PID.kP,drive.HEADING_PID.kI,drive.HEADING_PID.kD,0);

    }

    public void moveTo(Pose2d idealPose){
        Pose2d currentPose = drive.getPoseEstimate();
        double heading = atan2(idealPose.getX()-currentPose.getX(), idealPose.getY()-currentPose.getY());
        double speed = Math.hypot(translationalPID.calculate(currentPose.getX(), idealPose.getX()), translationalPID.calculate(currentPose.getY(), idealPose.getY()));
        double idealAngle = normalizeAngle(idealPose.getHeading());
        double currentAngle = normalizeAngle(currentPose.getHeading());
        double sign;
        if (clockwise(idealAngle, currentAngle)){
            sign = -1.0;
        }
        else{
            sign = 1.0;
        }
        double rotationSpeed = Math.abs(headingPID.calculate(idealAngle, currentAngle));

        double RF = Math.sin(heading)*speed + sign*rotationSpeed;
        double RB = Math.cos(heading)*speed + sign*rotationSpeed;
        double LF = Math.cos(heading)*speed - sign*rotationSpeed;
        double LB = Math.sin(heading)*speed - sign*rotationSpeed;

        drive.setMotorPowers(LF, LB, RB, RF);
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

}
