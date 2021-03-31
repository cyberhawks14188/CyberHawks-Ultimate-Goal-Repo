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
    double DoThetaSpeed;
    //Sets our Proportional and Derivative multipliers
    double speedPM = .013;
    double speedDM = .079;
    double thetaError;
    double thetaLastError;
    double thetaSpeedSetpoint;
    double lastTheta;
    double thetaSpeedCurrent;
    double thetaPM = .01;
    double thetaDM = .019;
    double thetaSpeedError;
    double thetaProportional;
    double thetaDerivitave;
    double thetaSpeed;

    public void SpeedCalc(double odoX, double odoY, double odoTheta, double time, double speedsetpoint, double thetaspeedsetpoint) {

        //Finds the difference in the thate position from last loop cycle to the current loop cycle
        thetaError = Math.abs(odoTheta - lastTheta);
        //Set lastTheta for use in next loop cycle
        lastTheta = odoTheta;
        //Finds the speed of theta for this loop cycle
        thetaSpeedCurrent = thetaError / (time - timePrevious);


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

        //theta speed PD
        thetaSpeedError = thetaspeedsetpoint - thetaSpeedCurrent;
        //theta speed proportional
        thetaProportional = thetaSpeedError * thetaPM;
        //theta speed dervitave
        thetaDerivitave = (thetaSpeedError - thetaLastError) * thetaDM;
        //theta last error for derivitave last loop
        thetaLastError = thetaSpeedError;
        //theta speed
        if(DoThetaSpeed == 1){
        thetaSpeed = thetaProportional + thetaDerivitave;
        }
        else{
            thetaSpeed = 0;
        }
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
        speed = Math.abs((speed + (speedDerivative + speedPorportional)));
        //Speed limits
        if (speed <= 0) {
            speed = 0;
        }
        if (speed >= 1) {
            speed = 1;
        }
    }

    //Returns our speed that will be applied to the motors
    public double SpeedReturn() {
        return speed;
    }

    //Return the robots current speed in in/s
    public double qCurrentSpeed() {
        return speedCurrent;
    }

    //Returns the distance traveled in the loop cycle
    public double DistanceDelta() {
        return distanceDelta;
    }

    public void MotionProfile(double speedtarget, double accelerationdistance, double deccelerationdistance, double slowmovementdistance,
     double distance, double distancefrom, double slowmovespeed, double thetaDecclerationDegree,
     double thetaSetpoint, double thetaTargetSpeed, double OdoTheta) {
        //Acceleration
        //Runs when our /acceleration distance is greater than our distance traveled

        //When our deceleration distance is greater than our distance from then we begin deceleration

        if (slowmovementdistance > distancefrom) {
            speedSetpoint = distancefrom * (slowmovespeed / slowmovementdistance);
        }
        else if (deccelerationdistance + slowmovementdistance > distancefrom) {
            //Uses y=mx + b to find what our speed setpoint during acceleration should be at a certain point
            speedSetpoint = (((speedtarget - slowmovespeed) / deccelerationdistance) * (distancefrom - slowmovementdistance)) + slowmovespeed;
        }
        else if ((accelerationdistance > distance - distancefrom) && (accelerationdistance > 0)) {
            //Uses y = mx + b to find what our speed setpoint during acceleration should be at a certain point
            speedSetpoint = (distance - distancefrom) * (speedtarget / accelerationdistance);
            //Sets a  minimum speed
            if (speedSetpoint <= .1) {
                speedSetpoint = .1;
            }
        }
        //If we are not accelerating or decelerating then go at our desired speed
        else {
            speedSetpoint = speedtarget;
        }
        //If do not want to accelerate or decelerate then immediately go to the desired speed
        if (accelerationdistance == 0 && deccelerationdistance == 0 && slowmovementdistance == 0) {
            speedSetpoint = speedtarget;
        }
        //returns the setpoint to be used in the speedCalc method
        if (distancefrom <= 2) {
            if(thetaDecclerationDegree > Math.abs(thetaSetpoint - OdoTheta)) {
                thetaSpeedSetpoint = (thetaTargetSpeed / thetaDecclerationDegree) * Math.abs(thetaSetpoint - OdoTheta);
            }
            DoThetaSpeed =1;
        }
        else{
            DoThetaSpeed = 0;
        }


    }
    public double speedSetpoint () {
        return speedSetpoint;
    }
    public double thetaSpeedSetpoint () {
        return thetaSpeedSetpoint;
    }
}