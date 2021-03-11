package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
public class DirectionCalcClass {
    TurnControl TurnClass = new TurnControl();
    //Declares Variables
    double distance;
    double distanceFrom;
    double distanceYLeg;
    double distanceXLeg;
    double distanceFromEndY;
    double distanceFromEndX;
    double LF_M_Direction;
    double LB_M_Direction;
    double RF_M_Direction;
    double RB_M_Direction;
    double yError;
    double xError;
    double yPorportional;
    double yDerivitive;
    double distanceFromSet;
    double yLastError;
    double xPorportional;
    double xDerivitive;
    double motorPowerRatio;
    double xLastError;
    double x;
    double y;
    double xsetpoint;
    double lastxsetpoint;
    double lastysetpoint;
    double ysetpoint;
    double endPointX;
    double endPointY;
    //Sets the distance we want our pure pursuit point to be set ahead of us
    double purePursuitDistance = 1.4;
    //Sets our PD multipliers
    double yPM = .0003;
    double yDM = .001;
    double xPM = .0003;
    double xDM = .001;
    public void DirectionCalc(double startpointx, double startpointy, double endpointx, double endpointy, double odoX, double odoY, double theta){
        //Sets the parameter to othe varible
        endPointX = endpointx;
        endPointY = endpointy;
        //Finds the distance from the start point to the end point in the x and y
        distanceYLeg = (startpointx - endpointx);
        distanceXLeg = (startpointy - endpointy);
        //Uses the pythagorean therom to find the overall distance that we have to travel from start point to end point
        distance = Math.hypot(distanceXLeg, distanceYLeg);
        //Find the diffrence between our current position and the end point
        distanceFromEndY = odoY - endpointy;
        distanceFromEndX = odoX - endpointx;
        //Uses the pythagorean therom to find how far we are from the end point
        distanceFrom = Math.hypot(distanceFromEndX, distanceFromEndY);
        distanceFromSet = 1;
        //Finds the point that the robot will be going to next
        //Finds this by calculating how far we are from the endpoint and how far ahead we want to look ahead using purePursuit Distance
        xsetpoint = endpointx - (((distanceFrom-purePursuitDistance)*(endpointx-startpointx))/distance);
        ysetpoint = endpointy - (((distanceFrom-purePursuitDistance)*(endpointy-startpointy))/distance);
        //If we want to maintain position then just hold the end setpoint
        if(startpointx == endpointx && startpointy == endpointy){
            xsetpoint = endpointx;
            ysetpoint = endpointy;
        }
        //Sets the last setpoinnt to the current setpoint
        lastxsetpoint = xsetpoint;
        lastysetpoint = ysetpoint;
        //Y PD
        yError = ysetpoint - odoY;
        yPorportional = yPM * yError;
        yDerivitive = (yError - yLastError)*yDM;
        yLastError = yError;
        y = yPorportional + yDerivitive;
        //X PD

        xError = xsetpoint - odoX;
        xPorportional = xPM * xError;
        xDerivitive = (xError - xLastError) * xDM;
        xLastError = xError;
        x = xPorportional + xDerivitive;
        //Makes sure our purepuresuit distance never goes backwards
        if(distanceFrom <= purePursuitDistance){
            xsetpoint = endpointx;
            ysetpoint = endpointy;
        }
        //When we are close to the desired point hold there
        if(distanceFrom <= .15){
            x = 0;
            y = 0;
            theta = 0;
        }
        //Motor Direction Equation

        //Find which direction should be turining based upon the x, y and theta correction
        LF_M_Direction = x + (-y + theta);
        LB_M_Direction = x - (-y - theta);
        RF_M_Direction = x - (-y + theta);
        RB_M_Direction = x + (-y - theta);
        //Finds what the highest motor power correction is and sets it to the bottom of the ratio
        motorPowerRatio = Math.max(Math.max(Math.abs(RF_M_Direction), Math.abs(RB_M_Direction)), Math.max(Math.abs(LF_M_Direction), Math.abs(LB_M_Direction)));
        //Puts each motor into a % of total power based on the highest motor power
        LF_M_Direction = LF_M_Direction/motorPowerRatio;
        LB_M_Direction = LB_M_Direction/motorPowerRatio;
        RF_M_Direction = RF_M_Direction/motorPowerRatio;
        RB_M_Direction = RB_M_Direction/motorPowerRatio;
    }
    public double XSetpointReturn(){return xsetpoint;}
    public double YSetpointReturn(){return ysetpoint;}
    public double DistanceFromSetReturn(){return distanceFromSet;}
    public double distanceReturn(){return distance;}
    public double distanceFromReturn(){return distanceFrom;}
    public double endpointXReturn(){return  endPointX;}
    public double endpointYReturn(){return endPointY;}
    public double LF_M_DirectionReturn(){return LF_M_Direction;}
    public double LB_M_DirectionReturn(){return LB_M_Direction;}
    public double RF_M_DirectionReturn(){return RF_M_Direction;}
    public double RB_M_DirectionReturn(){return RB_M_Direction;}
}
