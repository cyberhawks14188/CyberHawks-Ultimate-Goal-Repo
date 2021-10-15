package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;


public class VectorDriveClass {
    double motionVector = 0;
    double inputCombinedAngle = 0;
    double angleWithOffset = 0;
    double angleOffset;
    double xInputReversed;
    double xAfter, yAfter, zAfter;
    double RF_M, RB_M, LF_M, LB_M;

    public void VectorDriveMethod(double xInput, double yInput, double zInput, double actualZ){
        xInputReversed = -xInput;
        motionVector = Math.hypot(xInputReversed,yInput);
        inputCombinedAngle = Math.toDegrees(Math.atan(xInputReversed/yInput));
        angleWithOffset = inputCombinedAngle + actualZ;
        xAfter = Math.sin(inputCombinedAngle)*motionVector;
        yAfter = Math.cos(inputCombinedAngle)*motionVector;
        RF_M = xAfter - (yAfter + zInput);
        RB_M = xAfter + (yAfter - zInput);
        LF_M = xAfter + (yAfter + zInput);
        LB_M = xAfter - (yAfter - zInput);

    }
    public double RF_MRETURN(){ return RF_M;}
    public double RB_MRETURN(){ return RB_M;}
    public double LF_MRETURN(){ return LF_M;}
    public double LB_MRETURN(){ return LB_M;}
}

