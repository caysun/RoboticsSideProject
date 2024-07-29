package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerTurn {

    private double targetAngle;
    private double kP, kI, kD;
    private double integralSum = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private double lastError = 0.0;
    private boolean NotFirstCall = false;
    private double lastTime = 0.0;
    private double currentFilterEstimate = 0.0;
    private double previousFilterEstimate = 0.0;
    public PIDControllerTurn(double target, double P, double I, double D){
        targetAngle = target;
        kP = P;
        kI = I;
        kD = D;
    }

     public double update(double currentAngle){

        double motorPower = 0.0;
        //Value to mitigate noise by incorporating old and new values together
        double a = 0.1;

        //P
        double error = targetAngle - currentAngle;
        error = (error%360.0 + 360.0)%360.0;
        if(error > 180.0) error -= 360.0;
        //I
        integralSum += error * (runtime.milliseconds()-lastTime);
        if(Math.abs(error) < RobotAutoDriveByGyro_Linear.HEADING_ERROR_THRESHOLD){
            integralSum = 0.0;
        }
        integralSum = Math.abs(integralSum) * Math.signum(error);
        //D
        double slope = 0.0;
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * (error-lastError);
        previousFilterEstimate = currentFilterEstimate;

        if(NotFirstCall){
            slope = (currentFilterEstimate) / (runtime.milliseconds() - lastTime);
        }
        lastTime = runtime.milliseconds();
        lastError = error;
        NotFirstCall = true;

        //motor power
        motorPower += error * kP * error + integralSum * kI + slope * kD;
        return motorPower;
     }
}
