package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class DriveTrain {

    //Decalaring Variables to use in the method
    double speedSetPoint = 1;
    double LFM, LBM, RFM, RBM;
    double highestMotorPower;
    double leftStickCombined;
    double speed;
    public void DriveBase(double x, double y, double z, boolean gpadRightBumper){//parameters to get gamepad buttons to figure out where we want to go
        //Slows down drivebase if the right bumper is pressed for accurate movements if needed
        if (gpadRightBumper) {
            speedSetPoint = .5;
        } else {
            speedSetPoint = 1;
        }
        //Calculates variables to let us drive in the correct direction
        LFM = (-y) - ((-x) + (-z));
        LBM = (-y) + ((-x) - (-z));
        RFM = (-y) + ((-x) + (-z));
        RBM = (-y) - ((-x) - (-z));
        //We use highest motor power to make sure no wheel speed ever goes over 1 and lets us strafe accuratly
        highestMotorPower = Math.max(Math.max(Math.abs(LFM), Math.abs(LBM)), Math.max(Math.abs(RFM), Math.abs(RBM)));
        //Since we use highest motor power to make sure no motor power goes over 1, there will always be a motor going full power
        //We use the joysticks again to make apply a ratio and the set motor power
        leftStickCombined = Math.sqrt((x * x) + (y * y));
        if (Math.abs(z) < .01) {
            speed = leftStickCombined;
        } else if (Math.abs(x + y) < .02) {
            speed = Math.abs(z);
        } else {
            speed = (leftStickCombined + Math.abs(z)) / 2;
        }
        //Uses the calculations above to calculate the desired motor power
        LFM = ((LFM / highestMotorPower) * speed) * speedSetPoint;
        LBM = ((LBM / highestMotorPower) * speed) * speedSetPoint;
        RFM = ((RFM / highestMotorPower) * speed) * speedSetPoint;
        RBM = ((RBM / highestMotorPower) * speed) * speedSetPoint;

    }
    //returns motor power variables
    public double LFMReturn(){return LFM;}
    public double LBMReturn(){return LBM;}
    public double RFMReturn(){return RFM;}
    public double RBMReturn(){return RBM;}
}
