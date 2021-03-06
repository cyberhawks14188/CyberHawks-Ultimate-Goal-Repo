package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class  WobbleGoalArm {
    //Varibles for setting positions
    boolean WBControl = false;
    double WB_FSM = 5;
    //Position Varibles
    double wobbleEndSet = 0.4;
    double wobbleSet = .4;
    double gripperSet = .1;
    double wobbleError;
    double wobbleProportionalMulitplier = 2;
    double wobblePower;

    public void WobbleControl(double gpadlefttrigger, double wbpt){
        //finite State machine for the wobble goal.
        //We use the custom one button cycle to switch between each state.
        if (gpadlefttrigger > .05 && !WBControl) {
            if (WB_FSM < 4) {
                WB_FSM = WB_FSM + 1;
            } else {
                WB_FSM = 0;
            }
            WBControl = true;
        } else if (gpadlefttrigger < .05) {
            WBControl = false;
        }
        //State 0 is claw open and in grabbing position
        if (WB_FSM == 0) {
            wobbleEndSet = 2.1;
            gripperSet = .65;
        } else if (WB_FSM == 1) {//state 1 is grab wobble goal while still in down position
            gripperSet = .1;
        } else if (WB_FSM == 2) {//Brings wobble goal arm in stored position to drive to wall
            wobbleEndSet = .6;
        } else if (WB_FSM == 3) {//Brings wobble goal above wall, gripper still closed
            wobbleEndSet = 1;
        } else if (WB_FSM == 4) {// Gripper opens
            gripperSet = .65;
        } else if (WB_FSM == 5) {//Closes Claw
            gripperSet = .1;
        }
        //This slowly brings the arm to the endsetpoint to ensure little wear from arm bannging on the wheels
        if (wobbleEndSet > wobbleSet + .08) {
            wobbleSet = wobbleSet + .08;
        } else if (wobbleEndSet < wobbleSet - .08) {
            wobbleSet = wobbleSet - .08;
        } else {
            wobbleSet = wobbleEndSet;
        }
        //PID to control the wobble arm to go to desired set point
        wobbleError = wobbleSet - wbpt;
        wobblePower = wobbleError * wobbleProportionalMulitplier;
    }
    //method to use the wobble goal arm in autonomous
    public void WobbleAuto(double wbpt, double wobbleEndSet, double gripperset){
        //This slowly brings the arm to the endsetpoint to ensure little wear from arm bannging on the wheels
        if (wobbleEndSet > wobbleSet + .025) {
            wobbleSet = wobbleSet + .025;
        } else if (wobbleEndSet < wobbleSet - .025) {
            wobbleSet = wobbleSet - .025;
        } else {
            wobbleSet = wobbleEndSet;
        }
        //PID to control the wobble arm to go to desired set point
        gripperSet = gripperset;
        wobbleError = wobbleSet - wbpt;
        wobblePower = wobbleError * wobbleProportionalMulitplier;
    }
    //returns values in the variables when calles to the method
    public double wobblePowerReturn(){return wobblePower;}
    public double gripperSetReturn(){return gripperSet;}
    }
