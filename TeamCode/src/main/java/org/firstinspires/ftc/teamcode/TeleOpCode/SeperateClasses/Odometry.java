package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class Odometry {
    double deltaE1, deltaE2, deltaE3;
    double e1Previous, e2Previous, e3Previous;
    final double encoderWheelDistance = 14.42206;
    final double COUNTS_PER_INCH = 236.5;
    final double encoderWheelDistanceInch = encoderWheelDistance * COUNTS_PER_INCH;
    double thetaChange, thetaInRadians;
    double e2WithOffSet;
    double e2XOffSet = 1815;
    double yAverage;
    double yCoordinatePosition, xCoordinatePosition;
    double e1Current, e2Current, e3Current;
    double previousruntime = 0;

    public void OdometryCalc(double e1current, double e2current, double e3current, double runtime){
        //finds the change in the encoders from the last loop cycle
            e1Current = e1current;
            e2Current = e2current;
            e3Current = e3current;
            deltaE1 = e1Current - e1Previous;
            deltaE2 = e2Current - e2Previous;
            deltaE3 = e3Current - e3Previous;
            //calculates the angle in radians
            thetaChange = (deltaE1 - deltaE3) / encoderWheelDistanceInch;
            thetaInRadians = thetaInRadians + thetaChange;
            //calculates the e2 reading without the angle values affecting in it
            e2WithOffSet = deltaE2 - (thetaChange * e2XOffSet);
            //finds the average between the forward facing encoders to find the center of the robot
            yAverage = (deltaE1 + deltaE3) / 2;

            yCoordinatePosition = yCoordinatePosition + (yAverage * Math.sin(thetaInRadians)) + (e2WithOffSet * Math.cos(thetaInRadians));
            xCoordinatePosition = xCoordinatePosition + (yAverage * Math.cos(thetaInRadians)) - (e2WithOffSet * Math.sin(thetaInRadians));
            //sets varibles to the current encoder reading to set Delta variables in the next loop cycle
            e1Previous = e1Current;
            e2Previous = e2Current;
            e3Previous = e3Current;
            previousruntime = runtime;

    }
    double e2CenterOffSet = 7.21 * COUNTS_PER_INCH;//r
    double e2VertOffSet = 1815;//rb
    double vertHeadingPivotPoint;
    double HorisontalHeadingPivotPoint;
    public void RadiusOdometry(double e1current, double e2current, double e3current){
        e1Current = -e1current;
        e2Current = e2current;
        e3Current = -e3current;
        deltaE1 = e1Current - e1Previous;//ΔL
        deltaE2 = e2Current - e2Previous;//ΔB
        deltaE3 = e3Current - e3Previous;//ΔR
        thetaChange = (deltaE3 - deltaE1) / (2 * e2CenterOffSet);//Δ0
        thetaInRadians = thetaInRadians + thetaChange;
        if (thetaChange == 0){
            yCoordinatePosition = yCoordinatePosition + deltaE2;//Δx
            xCoordinatePosition = xCoordinatePosition + ((deltaE1 + deltaE3) / 2);//Δy
        }else{
            vertHeadingPivotPoint = (e2CenterOffSet*(deltaE1 + deltaE3)) / (deltaE3 - deltaE1);//rt
            HorisontalHeadingPivotPoint = (deltaE2 / thetaChange) - e2VertOffSet;//rs
            yCoordinatePosition = yCoordinatePosition + ((vertHeadingPivotPoint * (Math.cos(thetaChange) - 1)) + (HorisontalHeadingPivotPoint * Math.sin(thetaChange)));//Δx
            xCoordinatePosition = xCoordinatePosition + ((vertHeadingPivotPoint * Math.sin(thetaChange)) + (HorisontalHeadingPivotPoint * (1 - Math.cos(thetaChange))));//Δy
        }
        e1Previous = e1Current;
        e2Previous = e2Current;
        e3Previous = e3Current;
    }
    //returns the values to use in other classes
    public double odoXReturn(){return (-xCoordinatePosition/COUNTS_PER_INCH);}
    public double odoYReturn(){return yCoordinatePosition/COUNTS_PER_INCH;}
    public double thetaINRadiansReturn(){return thetaInRadians;}
    public double thetaInDegreesReturn(){return Math.toDegrees(thetaInRadians) % 360;}
}
