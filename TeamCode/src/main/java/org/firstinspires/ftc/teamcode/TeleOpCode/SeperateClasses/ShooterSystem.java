package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class ShooterSystem {
    //Sets varibles to use in the method
    boolean shooterControlBoolean;
    double SOTAngleSet = 1.03;
    double SOTAngleError;
    double SOTAngleLastError = 0;
    double SOTAngleDerivitveMultiplier = -5;
    double SOTAnglePropotionalMultiplier = -8;
    double SOTAnglePower;
    double shooterMotorSetpoint = 0;
    double shooterMotorCorrection = 0;
    double shooterMotorProportionalMultiplier = 15;
    double timePassed;
    double previousTime;
    double shooterMotorVelocity;
    double previousShooterMotorEncoder;
    double shooterMotorError;
    double shooterFSM = 1;
    // Shooter control method sets the shooter flywheel speed, runs the PID to make sure the flywheel is at the correct speed and adjusts the angle of the shooter
    public void shooterControl(boolean leftbumper, double shootermotorcurrent, double runtime, double sotanglecurrent, double intakepower){
        //Flywheel speed setpoint control. We use our custom one button on/off system to use the left bumper to set the shooter speed.
        if (leftbumper && !shooterControlBoolean) {
            if (shooterFSM < 3){
                shooterFSM = shooterFSM + 1;
            }else{
                shooterFSM = 0;
            }
            shooterControlBoolean = true;
        } else if (!leftbumper) {
            shooterControlBoolean = false;
        }
        if(shooterFSM == 0){
            shooterMotorSetpoint = 0;
            shooterMotorCorrection = 0;//we set both of these variables to ensure that neither one has power
        }else if(shooterFSM == 1){//Top Goal state
            if(intakepower == 0){
                shooterMotorSetpoint = 1900;//Shooter flywheel set point is 1900 encoder ticks per second
                timePassed = runtime - previousTime;
                previousTime = runtime;
                shooterMotorVelocity = Math.abs(shootermotorcurrent - previousShooterMotorEncoder) / timePassed;
                previousShooterMotorEncoder = shootermotorcurrent;
                shooterMotorError = shooterMotorSetpoint - shooterMotorVelocity;
                shooterMotorCorrection = shooterMotorError * shooterMotorProportionalMultiplier;
            }else{
                shooterMotorSetpoint = 0;
                shooterMotorCorrection = 0;//we set both of these variables to ensure that neither one has power
            }
            SOTAngleSet = 1.03;
        }else if (shooterFSM == 2){//Powershot state
            if(intakepower == 0) {
                shooterMotorSetpoint = 1200;//Shooter flywheel set point is 1900 encoder ticks per second
                timePassed = runtime - previousTime;
                previousTime = runtime;
                shooterMotorVelocity = Math.abs(shootermotorcurrent - previousShooterMotorEncoder) / timePassed;
                previousShooterMotorEncoder = shootermotorcurrent;
                shooterMotorError = shooterMotorSetpoint - shooterMotorVelocity;
                shooterMotorCorrection = shooterMotorError * shooterMotorProportionalMultiplier;
                SOTAngleSet = 1.275;
            }else{
                shooterMotorSetpoint = 0;
                shooterMotorCorrection = 0;//we set both of these variables to ensure that neither one has power
            }
        }
        //Shooter Angle PID Loop follow the setpoint set above
        SOTAngleError = SOTAngleSet - sotanglecurrent;
        SOTAnglePower = ((SOTAngleError * SOTAnglePropotionalMultiplier) + ((SOTAngleError - SOTAngleLastError)*SOTAngleDerivitveMultiplier));
        SOTAngleLastError = SOTAngleError;

    }
    //Method that we can use in autonomous
    public void ShooterControlAuto(double shootermotorcurrent, double runtime, double sotanglecurrent, double shootersetpoint, double sotangleset) {
        //Shooter Angle PID Loop follow the setpoint set above
        SOTAngleError = sotangleset - sotanglecurrent;
        SOTAnglePower = ((SOTAngleError * SOTAnglePropotionalMultiplier) + ((SOTAngleError - SOTAngleLastError)*SOTAngleDerivitveMultiplier));
        SOTAngleLastError = SOTAngleError;
        //Flywheel speed setpoint control. We use our custom one button on/off system to use the left bumper to set the shooter speed.

        //This is the Flywheels PID. This makes sure the Shooter speed is the same no matter the battery power
        if (shootersetpoint != 0) {
            timePassed = runtime - previousTime;
            previousTime = runtime;
            shooterMotorVelocity = Math.abs(shootermotorcurrent - previousShooterMotorEncoder) / timePassed;
            previousShooterMotorEncoder = shootermotorcurrent;
            shooterMotorError = shootersetpoint - shooterMotorVelocity;
            shooterMotorCorrection = shooterMotorError * shooterMotorProportionalMultiplier;

        }else{
            shooterMotorSetpoint = 0;
            shooterMotorCorrection = 0;//we set both of these variables to ensure that neither one has power
        }
    }
    //returns values to use when we call the methods
    public double shooterMotorPowerReturn(){return (shooterMotorSetpoint+shooterMotorCorrection)/2800;}
    public double sotAnglePowerReturn(){ return SOTAnglePower; }
    public double sotAngleSetReturn(){return SOTAngleSet;}
}
