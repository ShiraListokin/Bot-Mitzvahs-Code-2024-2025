package org.firstinspires.ftc.teamcode.assist;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.intake;
import org.firstinspires.ftc.teamcode.subSystems.current.slideStates;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public abstract class cycleAssistStates{

    //Subsystems
    protected intake in;
    protected slideStates slides;
    protected utilMovment movment;
    protected SampleMecanumDrive drive;
    ElapsedTime runtime;

    //Time
    private double time = -1;


    public cycleAssistStates(intake i, slideStates s, utilMovment m, SampleMecanumDrive sa, ElapsedTime r) {
        in = i;
        slides = s;
        movment = m;
        drive = sa;
        runtime = r;
    }


    protected double[] distanceNumbers(Pose2d pi, Pose2d pf){
        double xi = pi.getX();
        double yi = pi.getY();
        double thetai = pi.getHeading();

        double xf = pf.getX();
        double yf = pf.getY();
        double thetaf = pf.getHeading();

        double deltaX = xf-xi;
        double deltaY = yf-yi;

        double angleInBetween = angleBetween(thetaf, thetai);
        double distance = Math.hypot(deltaX, deltaY);
        return new double[] {distance, angleInBetween};//distance then angle
    }

    protected boolean checkIfInsideBox(double angle, double rotPrec, double dist, double transPrec){
        return ((angle < rotPrec) && (dist < transPrec));
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
}