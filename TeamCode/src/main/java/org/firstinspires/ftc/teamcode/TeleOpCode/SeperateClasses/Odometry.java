package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class Odometry {
    //declares varibles and sets needed numbers
    double deltaE1, deltaE2, deltaE3;
    double e1Previous, e2Previous, e3Previous;
    final double COUNTS_PER_INCH = 236.5;
    double thetaChange, thetaInRadians;
    double yCoordinatePosition, xCoordinatePosition;
    double e1Current, e2Current, e3Current;
    double e2CenterOffSet = 7.21 * COUNTS_PER_INCH;//r
    double e2VertOffSet = 1815;//rb
    double vertHeadingPivotPoint;
    double HorisontalHeadingPivotPoint;
    public void RadiusOdometry(double e1current, double e2current, double e3current){
        //writes parameters to variables for easy use in the method
        e1Current = -e1current;
        e2Current = e2current;
        e3Current = -e3current;
        //finds the difference in the encoders from the last loop cycle
        deltaE1 = e1Current - e1Previous;//ΔL
        deltaE2 = e2Current - e2Previous;//ΔB
        deltaE3 = e3Current - e3Previous;//ΔR
        //finds the angle the robot has changed in radians
        thetaChange = (deltaE3 - deltaE1) / (2 * e2CenterOffSet);//Δ0
        //using the calculated change in angle we calculate the angle the robot is facing in radians
        thetaInRadians = thetaInRadians + thetaChange;
        if (thetaChange == 0){//is the robot's we calculate the moved positions by adding the average to the encoder angles
            yCoordinatePosition = yCoordinatePosition + deltaE2;//Δx
            xCoordinatePosition = xCoordinatePosition + ((deltaE1 + deltaE3) / 2);//Δy
        }else{//is the robot's angle has changed we run calculations using sine and cosine
            //finds the pivot points that the robot moved upon to let us calculate where we had moved
            vertHeadingPivotPoint = (e2CenterOffSet*(deltaE1 + deltaE3)) / (deltaE3 - deltaE1);//finds pivot point to calculate for forward movement
            HorisontalHeadingPivotPoint = (deltaE2 / thetaChange) - e2VertOffSet;//finds pivot point for strafing
            //calculates the position of the robot to let us accurately go to a position
            yCoordinatePosition = yCoordinatePosition + ((vertHeadingPivotPoint * (Math.cos(thetaChange) - 1)) + (HorisontalHeadingPivotPoint * Math.sin(thetaChange)));//Δx
            xCoordinatePosition = xCoordinatePosition + ((vertHeadingPivotPoint * Math.sin(thetaChange)) + (HorisontalHeadingPivotPoint * (1 - Math.cos(thetaChange))));//Δy
        }
        //set the encoder to a varible to use in the next loop cycle to calculate the change in position
        e1Previous = e1Current;
        e2Previous = e2Current;
        e3Previous = e3Current;
    }
    //returns the values to use in other classes
    public double odoXReturn(){return (-xCoordinatePosition/COUNTS_PER_INCH);}
    public double odoYReturn(){return yCoordinatePosition/COUNTS_PER_INCH;}
    public double thetaINRadiansReturn(){return thetaInRadians;}
    public double thetaInDegreesReturn(){return Math.toDegrees(thetaInRadians);}//returns our angle in degrees with a value from 1-360
}
