package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.RingSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.ShooterSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.WobbleGoalArm;

import java.nio.file.DirectoryIteratorException;
import java.util.List;

@Autonomous

public class BlueAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    SpeedClass SpeedClass = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    WobbleGoalArm Wobble = new WobbleGoalArm();
    ShooterSystem Shooter = new ShooterSystem();
    RingSystem Stager = new RingSystem();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Uses Vuforia Developer Code
    private static final String VUFORIA_KEY = "AZickLn/////AAABmRdNRU8Vt0+EsSkecZ/dEtdwfmReQRmGjONFJw9IrZwj83V0JqVOw7lVMu8esNz/c2srCeQNiZSotXn5mKGHThTl4m0nN9xTmOVBgIkUOrtkA1rGeUkBw0dPy5AD8pk5L4Mv2yikiYUEXDVsPvVYjsp9p2+SHZNPXSBRL8OUPsUa+DpTQnpdRgtca4ZmRFGwUsfqkj/2pTz3/aS8KpFzZ6mjMVKJbJwiZnMhND5Bhy600+NNUNiTka0g6E+9lDEBQI5H0XVkEGCjHIFA0F8Z7L4iIZhotBPNB8kx3ep3MSRQSGg/yrzNIM4av2BqM2JVohuQFh2uSWyDJdgEwxtZ6drh3YZIa12CsuSHNkgaas2k";
    //Declares Varibles
    double breakout;
    double Detected;
    double startPointX;
    double startPointY;
    double lastEndPointY;
    double justTurn;
    double shooterSetpoint;
    double stopperSetpoint;
    double stagerSetpoint;
    double shooterAngleSetpoint;
    double wobbleSetpoint;
    double gripSetpoint;
    double intakeServoSetpoint;
    double intakeSetpoint;
    double timepassed;
    double lastEndPointX;
    double noDriveMotor;
    double xSetpoint;
    double ySetpoint;
    double turnIncriments;
    double thetaSetpoint;
    double loopcount;
    double accelerationDistance;
    double slowMoveSpeed;
    double decelerationDistance;
    double slowmovedistance;
    double thetaTargetSpeed;
    double thetaDeccelerationDegree;
    double targetSpeed;
    double stopProgram;
    double timepassed2;

    double action;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
/*
        //Intializes Vuforia
        initVuforia();
        //Intializes TensorFlow
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        while (isStarted() != true) {
            //Sets up a ratio for our webcam to look at
            //Allows our camera to focus only on the ring stack
            tfod.setZoom(3, 1.78);
            //Runs Tenosor Flow to detect the ring stack
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        //If we see 1 ring set Detected to 1
                        if (recognition.getLabel() == "Single") {
                            Detected = 1;
                        }
                        //If we see 4 rings set detected to 2
                        if (recognition.getLabel() == "Quad")
                            Detected = 2;
                    }

                    Telemetry();
                    telemetry.update();
                }
            }
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .32, .09);
            PowerSetting();
        }


 */
        waitForStart();
        action = 1;
        startPointX = 0;
        startPointY = 0;
        stopperSetpoint = .3;
        wobbleSetpoint = 1.4;
        shooterAngleSetpoint = 1.22;
        shooterSetpoint = 1700;
        gripSetpoint = .1;
        Detected = 2;
        robot.Ring1_CS.setGain(15); robot.Ring2_CS.setGain(15); robot.Ring3_CS.setGain(15);
        while(opModeIsActive() && stopProgram == 0) {
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors(); NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors(); NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();
            //Moves to first power shot shooting position
            if(action == 1){
                wobbleSetpoint = .6;
                    xSetpoint = 51; ySetpoint = -40; thetaSetpoint = 0; targetSpeed = 70; accelerationDistance = .5; decelerationDistance = 4;
                     slowmovedistance = 1.5; slowMoveSpeed = 1.5; thetaDeccelerationDegree = 2; thetaTargetSpeed = .3;

                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .2 && OdoClass.thetaInDegreesReturn() > -.2)){
                    StopMotors();
                    action = 2; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .2;
                }
                else{
                    breakout = 1;
                }
            }
            //Shoots 1st powershot
            else if(action == 2) {
                turnIncriments = .9;
                decelerationDistance = 2;
                stopperSetpoint = .5;
                stagerSetpoint = .45;
                if(timepassed <= getRuntime()){
                    action = 3; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    stopperSetpoint = .3;
                }
            }
            //Goes to 2nd power shot
            else if(action == 3){
                    xSetpoint = 51; ySetpoint = -33.1;  accelerationDistance = 0; decelerationDistance = 0; targetSpeed = 5;
                slowmovedistance = 2; slowMoveSpeed = 1; thetaDeccelerationDegree = 3; thetaTargetSpeed = .3;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .2 && OdoClass.thetaInDegreesReturn() > -.2)){
                    StopMotors();
                    action = 4; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .2;
                }
                else{
                    breakout = 1;
                }
            }
            //Shoots 2nd powershot
            else if(action == 4){
                decelerationDistance = 2;
                stopperSetpoint = .5;
                if(timepassed <= getRuntime()){
                    action = 5; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    stopperSetpoint = .3;
                }
            }
            //Goes to 3rd power shot
            else if(action == 5) {
                    xSetpoint = 51; ySetpoint = -28;  accelerationDistance = 0; decelerationDistance = 0; targetSpeed = 8;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < .2 && OdoClass.thetaInDegreesReturn() > -.2)){
                    StopMotors();
                    action = 6; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .2;
                }
                else{
                    breakout = 1;
                }
            }
            //Shoots 3rd powershot
            else if(action == 6) {
                stopperSetpoint = .5;
                decelerationDistance = 2;
                if(timepassed <= getRuntime()){
                    action = 7; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
            }
            //Goes to the 1st wobble goal
            else if(action == 7) {
                wobbleSetpoint = 1.8;
                
                if(Detected == 0){
                    xSetpoint = 30; ySetpoint = -31;  accelerationDistance = 1; decelerationDistance = 4; targetSpeed = 70;
                }
                if(Detected == 1){
                    xSetpoint = 30; ySetpoint = -31;  accelerationDistance = 1; decelerationDistance = 4; targetSpeed = 70;
                }
                if(Detected == 2){
                    xSetpoint = 108; ySetpoint = 27;  accelerationDistance = 1; decelerationDistance = 4; targetSpeed = 70;
                }
                stopperSetpoint = .3;
                stagerSetpoint = 0;
                if (DirectionClass.distanceFromReturn() <= .6 && breakout != 0){
                    StopMotors();
                    action = 8; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .5;
                }
                else{
                    breakout = 1;
                }
                }
            else if(action == 8){
                decelerationDistance = 10;  targetSpeed = 8; wobbleSetpoint = 2.1;
                if(robot.WB_PT.getVoltage() >= 2.04){
                    gripSetpoint = .65;
                }
                if(getRuntime() >= timepassed){
                    action = 9; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
            }

            else if(action == 9){
                wobbleSetpoint = 2.1;
                xSetpoint = 35; ySetpoint = -25; thetaSetpoint = 0; targetSpeed = 70; accelerationDistance = 1; decelerationDistance = 8; slowmovedistance = 1.5; intakeServoSetpoint = .6;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 10; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
                else{
                    breakout = 1;
                }
            }

            else if(action == 10){
                gripSetpoint = .65;
                xSetpoint = 18; ySetpoint = -10; thetaSetpoint = 0; targetSpeed = 40; accelerationDistance = 1; decelerationDistance = 8; slowmovedistance = 1.5; intakeServoSetpoint = .6;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 11; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
                else{
                    breakout = 1;
                }
            }

            else if(action == 11){
                thetaSetpoint = -87; targetSpeed = 4; accelerationDistance = 0; decelerationDistance = 0; slowmovedistance = 0;
                thetaDeccelerationDegree = 10; thetaTargetSpeed = 10;
                if (OdoClass.thetaInDegreesReturn() <= -84 && breakout != 0){
                    StopMotors();
                    action = 12; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 12){

                robot.LF_M.setPower(.55);
                robot.LB_M.setPower(.55);
                robot.RF_M.setPower(.55);
                robot.RB_M.setPower(.55);
                 thetaSetpoint = -90; targetSpeed = 0; accelerationDistance = 0; decelerationDistance = 0; slowmovedistance = 0;
                if (OdoClass.odoXReturn() >= 25.5 && breakout != 0){
                    StopMotors();
                    gripSetpoint = .1;

                    action = 13; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 1;
                }
                else{
                    breakout = 1;
                }
            }
            else  if(action == 13){
                if(timepassed <= getRuntime()){
                    wobbleSetpoint = 1;
                    action = 14; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    xSetpoint = OdoClass.odoXReturn();
                    ySetpoint = OdoClass.odoYReturn();
                }
            }
            else if(action == 14){
                thetaSetpoint = 0; targetSpeed = 6; accelerationDistance = 0; decelerationDistance = 0; slowmovedistance = 0;
                if (OdoClass.thetaInDegreesReturn() >= -5 && breakout != 0){
                    StopMotors();
                    action = 15; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 15){
                slowmovedistance = 5; slowMoveSpeed = 1.5; thetaDeccelerationDegree = 5; thetaTargetSpeed = 2;
                wobbleSetpoint = .6;
                xSetpoint = 30.5; ySetpoint = -2; thetaSetpoint = 10; targetSpeed = 20; accelerationDistance = 1; decelerationDistance = 1; slowmovedistance = .5;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 16; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 16){
                xSetpoint = 32.7; targetSpeed = 10; decelerationDistance = .25; slowmovedistance = .1; accelerationDistance = 0; intakeSetpoint = -.9; stagerSetpoint = .7;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 17; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .5;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 17){
                shooterAngleSetpoint = 1.13;
                targetSpeed = 3; decelerationDistance = 5; slowmovedistance =  10; intakeServoSetpoint = .72;
                if (Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red > .25 && breakout != 0){
                    StopMotors();
                    action = 18; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 1.25;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 18){
                if (timepassed <= getRuntime()){
                    StopMotors();
                    action = 19; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .4;
                }
            }

            else if(action == 19){
                thetaSetpoint = 10;  targetSpeed = 3; decelerationDistance = 5; slowmovedistance = 5; accelerationDistance = 0; intakeSetpoint = -1;
                if ((OdoClass.thetaInDegreesReturn() >= 9.5 && OdoClass.thetaInDegreesReturn() <= 10.5) && breakout != 0){
                    StopMotors();
                    action = 20; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + .7;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 20){
                StopMotors();
                if (timepassed <= getRuntime()){
                    StopMotors();
                    action = 21; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 2.5;
                }
            }
            else if(action == 21) {
                targetSpeed = 5;
                if (breakout == 0){
                    breakout = 1;
                    xSetpoint = xSetpoint + 1;
                }
                decelerationDistance = 10;
                slowmovedistance = 9;
                stagerSetpoint = .5;
                stopperSetpoint = .5;
                if(timepassed <= getRuntime()){
                    action = 22; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 2.5;
                }

            }
            else if (action == 22){
                stagerSetpoint = .5;
                intakeServoSetpoint = .78;
                if(timepassed <= getRuntime()){
                    intakeSetpoint = 0; shooterSetpoint = 0; stagerSetpoint = 0;
                    action = 23; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
            }
            else if(action == 23) {

                xSetpoint = 112; ySetpoint = 13; thetaSetpoint = 0;  accelerationDistance = 1; decelerationDistance = 3; targetSpeed = 70;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 24; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 1.5;
                }
                else{
                    breakout = 1;
                }
            }
            else if(action == 24){
                decelerationDistance = 10;  targetSpeed = 8; wobbleSetpoint = 2.1;
                if(robot.WB_PT.getVoltage() >= 2.04){
                    gripSetpoint = .65;
                }
                if(getRuntime() >= timepassed){
                    action = 25; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
            }
            else if(action == 25) {
                xSetpoint = 70; ySetpoint = 10; thetaSetpoint = 0;  accelerationDistance = 1; decelerationDistance = 10; targetSpeed = 50;
                if (DirectionClass.distanceFromReturn() <= .2 && breakout != 0){
                    StopMotors();
                    action = 26; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
                else{
                    breakout = 1;
                }
            }



            /*
            else if(action == 21){
                xSetpoint = 22; thetaSetpoint = 7;  targetSpeed = 10; decelerationDistance = 2; slowmovedistance = 0; accelerationDistance = 0; intakeSetpoint = .9;
                if (OdoClass.thetaInDegreesReturn() <= 6.5 && breakout != 0){
                    StopMotors();
                    action = 22; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 1.5;
                }
                else{
                    breakout = 1;
                }
            }


             */

            //If nothing else to do, stop the program
            else{
                stopProgram = 1;
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowmovedistance);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), shooterSetpoint, shooterAngleSetpoint);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), wobbleSetpoint, gripSetpoint);
            Stager.RingSystemAutonomous(intakeSetpoint, stopperSetpoint, stagerSetpoint, intakeServoSetpoint);
            if(robot.WB_PT.getVoltage() < 1.2){
                gripSetpoint = .1;
            }
            Telemetry();
            PowerSetting();
        }
        StopMotors();
        robot.WB_M.setPower(0);
        robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
        robot.SOT_M.setPower(0);
        robot.SOT_S.setPower(0);
        robot.STG_M.setPower(0);
        robot.STOP_S.setPosition(Stager.stopperSetReturn());
        while(opModeIsActive()){
            Telemetry();

            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        }
        }

        public void Telemetry () {
            telemetry.addData("Odo X", OdoClass.odoXReturn());
            telemetry.addData("Odo Y", OdoClass.odoYReturn());
            telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
            telemetry.addData("X", DirectionClass.XReturn());
            telemetry.addData("Y", DirectionClass.YReturn());
            telemetry.addData("Theta", TurnControl.theta);
            telemetry.addData("Distance", DirectionClass.distanceReturn());
            telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
            telemetry.addData("Speed", SpeedClass.SpeedReturn());
            telemetry.addData("time", getRuntime());
            telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
            telemetry.addData("XSetpoint", DirectionClass.XSetpointReturn());
            telemetry.addData("YSetpoint", DirectionClass.YSetpointReturn());
            telemetry.addData("LF_Power", robot.LF_M.getPower());
            telemetry.addData("RF_Power", robot.RF_M.getPower());
            telemetry.addData("LF_Direction", DirectionClass.LF_M_DirectionReturn());
            telemetry.addData("Motor Power Ratio", DirectionClass.motorPowerRatioReturn());
            telemetry.addData("WB PT", robot.WB_PT.getVoltage());
            telemetry.addData("Shooter Motor", robot.SOT_M.getPower());
            telemetry.addData("STG_M", robot.STG_M.getPower());
            telemetry.addData("Action", action);
            telemetry.update();
        }
    public void Movement (double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree,double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowmovedistance){
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint , OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowmovedistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint, SpeedClass.thetaSpeedSetpoint());
    }

        public void PowerSetting () {

            robot.WB_M.setPower(Wobble.wobblePowerReturn());
            robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
            robot.SOT_M.setPower(Shooter.shooterMotorPowerReturn());
            robot.SOT_S.setPower(Shooter.sotAnglePowerReturn());
            robot.STG_M.setPower(Stager.stagerPowerRetun());
            robot.STOP_S.setPosition(Stager.stopperSetReturn());
            robot.IN_S.setPosition(Stager.intakePositionReturn());
            robot.IN_M.setPower(Stager.intakePowerReturn());
            robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .1));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .1));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .1));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .1));
        }
        public void StopMotors () {
            robot.LF_M.setPower(0);
            robot.LB_M.setPower(0);
            robot.RF_M.setPower(0);
            robot.RB_M.setPower(0);
        }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.825f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    }
    /*
            else if(action == 2.5 || action == 2){
                StopMotors();
                action = 2.5;
                if(breakout == 1){
                    timepassed = getRuntime() + 3;
                    breakout = 0;
                }
                if(timepassed <= getRuntime()){
                    action = 3; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn();
                }

            }
            if(action == 1 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                shooterAngleSetpoint = 1.12;
                gripSetpoint = .1;
                xSetpoint = 15; ySetpoint = 15; thetaSetpoint = 0; targetSpeed = 20; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if(action == 1){
                StopMotors();
                action = 2; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 2 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = 19; ySetpoint = -20; thetaSetpoint = 0; targetSpeed = 20; accelerationDistance = 0; decelerationDistance = 0; breakout = 1;
            }

            else if(action == 2){
                StopMotors();
                action = 3; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 3 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = -30; ySetpoint = -15; thetaSetpoint = 0; targetSpeed = 10; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 3){
                StopMotors();
                action = 4; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 4 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = -30; ySetpoint =-25; thetaSetpoint = 0; targetSpeed = 50; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 4){
                StopMotors();
                action = 5; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 5 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = 30; ySetpoint = -25; thetaSetpoint = 0; targetSpeed = 30; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 5){
                StopMotors();
                action = 6; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 6 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = 40; ySetpoint = -30; thetaSetpoint = 0; targetSpeed = 14; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 6){
                StopMotors();
                action = 7; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 7 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = 0; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 20; accelerationDistance = 1.3; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 7){
                StopMotors();
                action = 8; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 8){
                stopProgram = 1;
            }


            if(opModeIsActive()){
                shooterAngleSetpoint = 1.12;
                gripSetpoint = .1;
                xSetpoint = 0; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 17; accelerationDistance = 0; decelerationDistance = 10; breakout = 1;
            }
            else if(action == 1){
                StopMotors();
                action = 2; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            */