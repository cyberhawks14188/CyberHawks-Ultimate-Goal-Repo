package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
public class SpeedClass {
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    //Declares Varibles
    double speedCurrent;
    double lastOdoX;
    double lastOdoY;
    double positionErrorX;
    double positionErrorY;
    double distanceDelta;
    double timePrevious;
    double speedLastError;
    double speedError;
    double speedPorportional;
    double speedDerivative;
    public double speed;
    public double speedSetpoint;
    //Sets our Proportional and Derivative multipliers
    double speedPM = .015;
    double speedDM = .055;

    public void SpeedCalc(double odoX, double odoY, double time, double speedsetpoint) {

            //Finds the difference between loop cycles in position
            //Uses absolute value since distances can't be negative
            positionErrorX = Math.abs(odoX - lastOdoX);
            positionErrorY = Math.abs(odoY - lastOdoY);
            //Sets our current position to our last position
            lastOdoX = odoX;
            lastOdoY = odoY;
            //Finds the hypotenuse/distance traveled in loop cycle by using the pythagrioum therom
            distanceDelta = Math.hypot(positionErrorX, positionErrorY);
            //Find the robots in/s, by finding the time passed in the loop cycle and putting the distance traveled to a ratio over 1
            speedCurrent = distanceDelta / (time - timePrevious);
            //Sets the previous time to our current time
            timePrevious = time;



        //Speed PD
        //Compares the desired speed to our current
        speedError = speedsetpoint - speedCurrent;
        //Speed proportional
        speedPorportional = speedError * speedPM;
        //Speed Derivative
        speedDerivative = (speedError - speedLastError) * speedDM;
        //Sets our last error to our current error
        speedLastError = speedError;
        //Speed at which the motor %'s will be going
        //We add speed to speed to allows the robot to always incress speed if it is going to slow
        speed = Math.abs(speed + ((speedDerivative + speedPorportional)));
        //Speed limits
        if (speed <= 0){
            speed = 0;
        }
        if(speed >= 1){
            speed = 1;
        }
    }
    //Returns our speed that will be applied to the motors
    public double SpeedReturn(){return speed;}
    //Return the robots current speed in in/s
    public double CurrentSpeed(){return speedCurrent;}
    //Returns the distance traveled in the loop cycle
    public double DistanceDelta(){return distanceDelta;}

    public double MotionProfile(double speedtarget, double accelerationdistance, double deccelerationdistance, double distance,  double distancefrom) {
        //Acceleration
        //Runs when our acceleration distance is greater than our distance traveled

        //When our deceleration distance is greater than our distance from then we begin deceleration
        if (deccelerationdistance > distancefrom) {
            //Uses y=mx + b to find what our speed setpoint during acceleration should be at a certain point
            speedSetpoint = distancefrom * (speedtarget / deccelerationdistance);
        }
        else if (accelerationdistance > distance - distancefrom) {
            //Uses y = mx + b to find what our speed setpoint during acceleration should be at a certain point
            speedSetpoint = (distance - distancefrom) * (speedtarget / accelerationdistance);
            //Sets a  minimum speed
            if(speedSetpoint <= .1){
                speedSetpoint = .1;
            }
        }
        //If we are not accelerating or decelerating then go at our desired speed
        else{
            speedSetpoint = speedtarget;
        }
        //If do not want to accelerate or decelerate then immediately go to the desired speed
        if(accelerationdistance == 0 & deccelerationdistance == 0){
            speedSetpoint = speedtarget;
        }
        //returns the setpoint to be used in the speedCalc method
    return speedSetpoint;
    }

}