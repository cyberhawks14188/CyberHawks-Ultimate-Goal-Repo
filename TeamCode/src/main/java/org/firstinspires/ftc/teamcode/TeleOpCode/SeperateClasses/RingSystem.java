package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class RingSystem {
    //variables to control the Finite State Machine
    boolean stagerControl = false;
    double ringSystemFSM;
    //variables to control motors or servos
    double intakePower;
    double stagerPower;
    double stopperSet;
    public void RingSystemControl(boolean gpadA, boolean gpadB, double colorSensor1, double colorSensor2, double colorSensor3, boolean gpadback){

        //uses a Finite State Machine to turn the intake and stager motors on or off and set the position of the stopper servo
        //to set what state the FSM is in, we use our 1 button function, The function uses a boolean to tell us if the button was pressed last loop cycle
        //if the button was't and it is now: change the state we are in, Then repeat until the program shuts off
        if (gpadA && !stagerControl) {
            if (intakePower == 0) {
                ringSystemFSM = 1;
            } else {
                ringSystemFSM = 2;
            }
            stagerControl = true;
        } else if (!gpadA) {
            stagerControl = false;
        }
        //stage 0 shuts all motors off and this stager is starting stage
        if (ringSystemFSM == 0) {
            intakePower = 0;
            stagerPower = 0;
        }
        //stage 1 is intaking stage. Sets ring stopper to closed and intakes until sensors see that there is 3 rings in robot.
        if (ringSystemFSM == 1) {
            stopperSet = .3;
            if (colorSensor1 > .05 && colorSensor2 > .25 && colorSensor3 > .25) {
                intakePower = 0;
                stagerPower = 0;
                ringSystemFSM = 2;
            } else {
                intakePower = -1;
                stagerPower = -1;
            }
        }
        //stage 2 is shooting stage. If button b is pressed rings shoot other wise motors are off
        if (ringSystemFSM == 2) {
            if (gpadB) {
                stopperSet = .5;
                stagerPower = -.9;
            } else {
                stagerPower = 0;
            }
            intakePower = 0;
        }
        if (gpadback) {
            intakePower = 1;
        }
    }
    public void RingSystemAuto(double ringSystemFSM, double colorSensor1, double colorSensor2, double colorSensor3){
        //uses a Finite State Machine to turn the intake and stager motors on or off and set the position of the stopper servo
        //stage 0 shuts all motors off and this stager is starting stage
        if (ringSystemFSM == 0) {
            intakePower = 0;
            stagerPower = 0;
        }
        //stage 1 is intaking stage. Sets ring stopper to closed and intakes until sensors see that there is 3 rings in robot.
        if (ringSystemFSM == 1) {
            stopperSet = .3;
            if (colorSensor1 > .05 && colorSensor2 > .25 && colorSensor3 > .25) {
                intakePower = 0;
                stagerPower = 0;
            } else {
                intakePower = -1;
                stagerPower = -1;
            }
        }
        //stage 2 is shooting stage.
        if (ringSystemFSM == 2) {
            stopperSet = .5;
            stagerPower = -.9;
            intakePower = 0;
        }
    }
    //returns the variables for the the intake and stager power and the stopper position
    public double intakePowerReturn(){return intakePower;}
    public double stagerPowerRetun(){return -stagerPower;}
    public double stopperSetReturn(){return stopperSet;}
}
