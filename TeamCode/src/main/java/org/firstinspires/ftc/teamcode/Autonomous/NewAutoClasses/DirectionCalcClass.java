package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
public class DirectionCalcClass {
    TurnControl TurnClass = new TurnControl();
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
    double yPM = .001;
    double yDM = .006;
    double xPM = .001;
    double xDM = .006;
    public void DirectionCalc(double startpointx, double startpointy, double endpointx, double endpointy, double odoX, double odoY, double theta){
        endPointX = endpointx;
        endPointY = endpointy;
        distanceYLeg = (startpointx - endpointx);
        distanceXLeg = (startpointy - endpointy);
        distance = Math.hypot(distanceXLeg, distanceYLeg);
        distanceFromEndY = odoY - endpointy;
        distanceFromEndX = odoX - endpointx;
        distanceFrom = Math.hypot(distanceFromEndX, distanceFromEndY);
        //Y PD

        xsetpoint = endpointx - (((distanceFrom-1)*(endpointx-startpointx))/distance);
        ysetpoint = endpointy - (((distanceFrom-1)*(endpointy-startpointy))/distance);
        if(startpointx == endpointx){
            xsetpoint = endpointx;
        }
        if(startpointy == endpointy){
            ysetpoint = endpointy;
        }
        lastxsetpoint = xsetpoint;
        lastysetpoint = ysetpoint;
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
        if(distanceFrom <= .15){
            x = 0;
            y = 0;
            theta = 0;
        }


        LF_M_Direction = x + (-y + theta);
        LB_M_Direction = x - (-y - theta);
        RF_M_Direction = x - (-y + theta);
        RB_M_Direction = x + (-y - theta);
        motorPowerRatio = Math.max(Math.max(Math.abs(RF_M_Direction), Math.abs(RB_M_Direction)), Math.max(Math.abs(LF_M_Direction), Math.abs(LB_M_Direction)));
        LF_M_Direction = LF_M_Direction/motorPowerRatio;
        LB_M_Direction = LB_M_Direction/motorPowerRatio;
        RF_M_Direction = RF_M_Direction/motorPowerRatio;
        RB_M_Direction = RB_M_Direction/motorPowerRatio;
    }
    public double XSetpointReturn(){return xsetpoint;}
    public double YSetpointReturn(){return ysetpoint;}
    public double distanceReturn(){return distance;}
    public double distanceFromReturn(){return distanceFrom;}
    public double endpointXReturn(){return  endPointX;}
    public double endpointYReturn(){return endPointY;}
    public double LF_M_DirectionReturn(){return LF_M_Direction;}
    public double LB_M_DirectionReturn(){return LB_M_Direction;}
    public double RF_M_DirectionReturn(){return RF_M_Direction;}
    public double RB_M_DirectionReturn(){return RB_M_Direction;}
}
