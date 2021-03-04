package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    double timepassed;
    double lsstEndPointX;
    double noDriveMotor;
    double loopcount;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
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
        waitForStart();
        startPointX = 0;
        startPointY = 0;
        breakout = 1;
        //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(47, 18, 0, 45, 2, 8);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.5, .08);
            Telemetry();
            PowerSetting();
            breakout = 0;
        }
        StopMotors();
        sleep(400);
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        if (Detected == 0) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(64, 18, 0, 30, 2, 9);
                Telemetry();
                Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.5, .08);
                PowerSetting();
                breakout = 0;

            }
        }
        if (Detected == 1) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(86, -8, 0, 45, 2, 9);
                Telemetry();
                Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.5, .08);
                PowerSetting();
                breakout = 0;
            }
        }

        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        if (Detected == 2) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(112, 18, 0, 45, 2, 9);
                Telemetry();
                Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.5, .08);
                PowerSetting();
                breakout = 0;
            }
        }


        StopMotors();
        timepassed = getRuntime() + 1;
        noDriveMotor = 1;
        while (timepassed > getRuntime()) {
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.8, .1);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            PowerSetting();
        }
        noDriveMotor = 0;
        startPointX = DirectionClass.endpointXReturn();
        startPointY = DirectionClass.endpointYReturn();
        breakout = 1;
        while (breakout != 0 && opModeIsActive()) {
            Movement(startPointX, startPointY, 0, .3, 0, 0);
            if (robot.WB_PT.getVoltage() <= 2) {
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .1);
            } else {
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .65);
                breakout = 0;
            }
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            Telemetry();
            PowerSetting();
        }
        StopMotors();
        noDriveMotor = 1;
        timepassed = getRuntime() + 2;
        while (timepassed > getRuntime()) {
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .6, .65);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            PowerSetting();
        }
        noDriveMotor = 0;
        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(59, -16, 0, 30, 6, 9);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .6, .08);
            Telemetry();
            PowerSetting();
            breakout = 0;
        }
        noDriveMotor = 1;
        StopMotors();
        timepassed = getRuntime() + 4;
        while (timepassed > getRuntime()) {
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .6, .65);
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            PowerSetting();
        }
        noDriveMotor = 0;
        StopMotors();
        startPointX = DirectionClass.endpointXReturn();
        startPointY = DirectionClass.endpointYReturn();
        noDriveMotor = 1;
        timepassed = getRuntime() + 4;
        while (timepassed > time) {
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 2000, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .6, .08);
            Stager.RingSystemAuto(2, 0, 0, 0);
            Telemetry();
            PowerSetting();
            StopMotors();
        }
        noDriveMotor = 0;
        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(40, -40, 0, 28, 2, 5);
            Telemetry();
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 0, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 1.5, .08);
            PowerSetting();
            breakout = 0;
        }
        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(15, -40, 0, 24, 2, 5);
            Telemetry();
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 0, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .65);
            PowerSetting();
            breakout = 0;
        }
        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        justTurn = 1;
        while ((OdoClass.thetaInDegreesReturn() >= -86 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(15, -40, -90, 10, 0, 0);
            Telemetry();
            Shooter.ShooterControlAuto(robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage(), 0, 1.12);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .08);
            PowerSetting();
            breakout = 0;
        }
        sleep(1000);
        StopMotors();
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = robot.LF_M.getCurrentPosition() + 2000;
        while(breakout >= robot.LF_M.getCurrentPosition()){
        robot.LF_M.setPower(.2);
        robot.LB_M.setPower(.2);
        robot.RF_M.setPower(.2);
        robot.RB_M.setPower(.2);
    }
        StopMotors();
        timepassed = getRuntime() + 1;
        noDriveMotor = 1;
        while (timepassed > time) {
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1 ,.08);
            PowerSetting();
        }
        StopMotors();
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
            robot.STOP_S.setPosition(Stager.stopperSetReturn());
            robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .15));
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