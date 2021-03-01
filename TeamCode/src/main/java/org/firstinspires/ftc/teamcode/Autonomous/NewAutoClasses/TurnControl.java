package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;

public class TurnControl {
    double thetaError;
    double thetaProportionalMultiplier = .00002;
    double thetaProportional;
    public double theta;
    double thetaSetPoint = 0;
    double thetaLastError = 0;
    double thetaDerivativeMultiplier = .00014;
    double thetaDerivative;
    public double turnControl(double thetaendsetpoint, double thetaindegrees, double turnincrements){
        if (thetaendsetpoint > thetaSetPoint + turnincrements){
            thetaSetPoint = thetaSetPoint + turnincrements;
        }else if(thetaendsetpoint < thetaSetPoint - turnincrements){
            thetaSetPoint = thetaSetPoint - turnincrements;
        }
        else{
            thetaSetPoint = thetaendsetpoint;
        }
        thetaError = thetaSetPoint - thetaindegrees;
        thetaProportional = thetaError * thetaProportionalMultiplier;
        thetaDerivative = (thetaError - thetaLastError)* thetaDerivativeMultiplier;
        thetaLastError = thetaError;
        theta = thetaProportional + thetaDerivative;
        return theta;
    }
}