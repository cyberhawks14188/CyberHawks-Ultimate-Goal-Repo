package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.RingSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.ShooterSystem;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.WobbleGoalArm;

import java.util.List;

@Autonomous

public class NewTurnProgramTest extends LinearOpMode {
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
    double intakeSetpoint;
    double timepassed;
    double lastEndPointX;
    double noDriveMotor;
    double xSetpoint;
    double ySetpoint;
    double thetaSetpoint;
    double loopcount;
    double accelerationDistance;
    double decelerationDistance;
    double targetSpeed;
    double stopProgram;
    double timepassed2;

    double action;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        action = 1;
        startPointX = 0;
        startPointY = 0;
        stopperSetpoint = .3;
        wobbleSetpoint = 1.4;
        shooterAngleSetpoint = 1.22;
        shooterSetpoint = 1700;
        gripSetpoint = .1;
        while(opModeIsActive() && stopProgram == 0) {
            //First move
            if(action == 1){
                xSetpoint = 60; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 30; accelerationDistance = 1; decelerationDistance = 6;
                if (DirectionClass.distanceFromReturn() <= 1 && breakout != 0){
                    StopMotors();
                    action = 2; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                }
                else{
                    breakout = 1;
                }

            }

            //Goes to target position
            else if(action == 2){
                xSetpoint = 0; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 30; accelerationDistance = 1; decelerationDistance = 6;
                if (DirectionClass.distanceFromReturn() <= 1 && breakout != 0){
                    StopMotors();
                    action = 3; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
                    timepassed = getRuntime() + 2;
                }
                else{
                    stopProgram = 1;
                }
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, accelerationDistance, decelerationDistance);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), shooterSetpoint, shooterAngleSetpoint);
            //Stager.RingSystemAuto();
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), wobbleSetpoint, gripSetpoint);
            Stager.RingSystemAutonomous(intakeSetpoint, stopperSetpoint, stagerSetpoint);
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
            telemetry.addData("Distance", DirectionClass.distanceReturn());
            telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
            telemetry.addData("Speed", SpeedClass.SpeedReturn());
            telemetry.addData("Current Speed", SpeedClass.CurrentSpeed());
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
            telemetry.addData("Action", action);
            telemetry.update();
        }
        public void Movement ( double endpointx, double endpointy, double thetasetpoint,
        double targetspeed, double accelerationdistance, double deccelerationdistance){
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta, thetasetpoint, OdoClass.thetaINRadiansReturn());
            SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
            SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
            if(justTurn == 1){
                TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 3);
            }
            else{
                TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), .6);
            }
            telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
        }
        public void PowerSetting () {

            robot.WB_M.setPower(Wobble.wobblePowerReturn());
            robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
            robot.SOT_M.setPower(Shooter.shooterMotorPowerReturn());
            robot.SOT_S.setPower(Shooter.sotAnglePowerReturn());
            robot.STG_M.setPower(Stager.stagerPowerRetun());
            robot.STOP_S.setPosition(Stager.stopperSetReturn());
            robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .21));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .21));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .21));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .21));
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