package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;

public class TurnControl {
    //Declares Varibles
    double thetaError;
    double thetaProportionalMultiplier = .00002;
    double thetaProportional;
    public double theta;
    double thetaSetPoint = 0;
    double thetaLastError = 0;
    double thetaDerivativeMultiplier = .00014;
    double thetaDerivative;
    public double turnControl(double thetaendsetpoint, double thetaindegrees, double turnincrements){
        //Turns the robots in incriments instead of going straight to the end setpoint
        //If our endsepoint is greater then where we want to go plus our turn incriment
        if (thetaendsetpoint > thetaSetPoint + turnincrements){
            //We then add to our current setpoint that we follow
            thetaSetPoint = thetaSetPoint + turnincrements;
        //If we are trying to go the other direction then subtract
        }else if(thetaendsetpoint < thetaSetPoint - turnincrements){
            thetaSetPoint = thetaSetPoint - turnincrements;
        }
        //If our setpoint = our endsetpoint then maintain
        else{
            thetaSetPoint = thetaendsetpoint;
        }
        //Theta PD
        thetaError = thetaSetPoint - thetaindegrees;
        thetaProportional = thetaError * thetaProportionalMultiplier;
        thetaDerivative = (thetaError - thetaLastError)* thetaDerivativeMultiplier;
        thetaLastError = thetaError;
        theta = thetaProportional + thetaDerivative;
        return theta;
    }
}