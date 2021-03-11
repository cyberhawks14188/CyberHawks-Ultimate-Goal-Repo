package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        */
        waitForStart();
        action = 1;
        startPointX = 0;
        startPointY = 0;
        wobbleSetpoint = 1;
        while(opModeIsActive() && stopProgram == 0) {
            if(action == 1 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                shooterAngleSetpoint = 1.12;
                gripSetpoint = .15;
                shooterSetpoint = 2000;
                xSetpoint = 40; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 30; accelerationDistance = 4; decelerationDistance = 8; breakout = 1;
            }
            else if(action == 1){
                StopMotors();
                action = 2; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 2 && (robot.WB_PT.getVoltage() >= 2 || breakout == 0)){
                wobbleSetpoint = 2.1;
                xSetpoint = 40; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 2; accelerationDistance = 0; decelerationDistance = 0; breakout = 1;
            }
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
            else if(action == 3 && (DirectionClass.distanceFromReturn() >= .6 || breakout == 0)){
                xSetpoint = 0; ySetpoint = 0; thetaSetpoint = 0; targetSpeed = 30; accelerationDistance = 4; decelerationDistance = 8; breakout = 1;
            }
            else if (action == 3){
                action = 4; startPointX = OdoClass.odoXReturn(); startPointY = OdoClass.odoYReturn(); breakout = 0;
            }
            else if(action == 4){
                stopProgram = 1;
            }

            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, accelerationDistance, decelerationDistance);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), shooterSetpoint, shooterAngleSetpoint);
            //Stager.RingSystemAuto();
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), wobbleSetpoint, gripSetpoint);

            Telemetry();
            PowerSetting();
        }
        StopMotors();
        while(opModeIsActive()){
            Telemetry();
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
            telemetry.addData("WB PT", robot.WB_PT.getVoltage());
            telemetry.addData("Shooter Motor", robot.SOT_M.getPower());
            telemetry.update();
        }
        public void Movement ( double endpointx, double endpointy, double thetasetpoint,
        double targetspeed, double accelerationdistance, double deccelerationdistance){
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
            SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
            SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
            if(justTurn == 1){
                TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 3);
            }
            else{
                TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 1);
            }
            telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
        }
        public void PowerSetting () {
        if(noDriveMotor == 1){
            robot.WB_M.setPower(Wobble.wobblePowerReturn());
            robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
            robot.SOT_M.setPower(Shooter.shooterMotorPowerReturn());
            robot.SOT_S.setPower(Shooter.sotAnglePowerReturn());
            robot.STG_M.setPower(Stager.stagerPowerRetun());
            robot.STOP_S.setPosition(Stager.stopperSetReturn());
        }
        else{
            robot.WB_M.setPower(Wobble.wobblePowerReturn());
            robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
            robot.SOT_M.setPower(Shooter.shooterMotorPowerReturn());
            robot.SOT_S.setPower(Shooter.sotAnglePowerReturn());
            robot.STG_M.setPower(Stager.stagerPowerRetun());
            robot.STOP_S.setPosition(Stager.stopperSetReturn());robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .19));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .19));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .19));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() + .19));
        }

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