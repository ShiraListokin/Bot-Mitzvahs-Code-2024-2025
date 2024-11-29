package org.firstinspires.ftc.teamcode.subSystems.current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.roadRunner.SampleMecanumDrive;

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
