package org.firstinspires.ftc.teamcode.subSystems.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.current.utilMovment;

public class utilMovmentAuto extends utilMovment {

    private int step = 0;
    private Pose2d[] currentPoses;

    private double transprec;

    private double rotprec;
    public utilMovmentAuto(SampleMecanumDrive drive1){
        super(drive1);
    }

    public void trajectory (Pose2d[] poses, double translationalPrecition, double rotationalPrecition){
        currentPoses = poses;
        transprec = translationalPrecition;
        rotprec = rotationalPrecition;
        step = 0;
    }

    public double[] update(){
        moveTo(currentPoses[step]);

        Pose2d currentPose = drive.getPoseEstimate();
        double xi = currentPose.getX();
        double yi = currentPose.getY();
        double thetai = currentPose.getHeading();

        double xf = currentPoses[step].getX();
        double yf = currentPoses[step].getY();
        double thetaf = currentPoses[step].getHeading();

        double deltaX = xf-xi;
        double deltaY = yf-yi;

        double angleInBetween = angleBetween(thetaf, thetai);
        double distance = Math.hypot(deltaX, deltaY);

        if(distance < transprec && angleInBetween < rotprec){
            step++;
        }
        return new double[] {step, xi, yi, thetai};
    }
}
