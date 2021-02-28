package org.firstinspires.ftc.teamcode.Autonomous;
import android.graphics.Path;
import android.util.Log;

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
    double lsstEndPointX;
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
        }
        waitForStart();
        startPointX = 0;
        startPointY = 0;
        breakout = 1;
        //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(47, 18, 0, 30, 6, 9);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), .6,.1 );
            Telemetry();
            PowerSetting();
            breakout = 0;
        }

        StopMotors();
        sleep(250);
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        if (Detected == 0) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(64, 18, 0, 30, 6, 9);
                Telemetry();
                PowerSetting();
                breakout = 0;
            }
        }
        if (Detected == 1) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(90, -8, 0, 45, 6, 9);
                Telemetry();
                PowerSetting();
                breakout = 0;
            }
        }

        StopMotors();
        sleep(250);
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;
        if (Detected == 2) {
            //(DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())
            while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
                Movement(112, 18, 0, 45, 6, 9);
                Telemetry();
                PowerSetting();
                breakout = 0;
            }
        }

        StopMotors();
        sleep(750);
        startPointX = DirectionClass.endpointXReturn();
        startPointY = DirectionClass.endpointYReturn();
        breakout = 1;
        while (breakout != 0) {
            Movement(startPointX, startPointY, 0, 45, 6, 9);
            if(robot.WB_PT.getVoltage() >= 2){
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .1);
            }
            else{
                Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 2.1, .65);
                breakout = 0;
            }
            Telemetry();
            PowerSetting();
        }
        StopMotors();
        sleep(500);
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        while ((DirectionClass.distanceFromReturn() >= 1 && opModeIsActive()) || (breakout == 1 && opModeIsActive())) {
            Movement(60, -16, 0, 30, 6, 9);
            Wobble.WobbleAuto(robot.WB_PT.getVoltage(), 3.5, .1);
            Telemetry();
            PowerSetting();
            breakout = 0;
        }
        StopMotors();
        sleep(250);
        startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        breakout = 1;

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
            telemetry.update();
        }
        public void Movement ( double endpointx, double endpointy, double thetasetpoint,
        double targetspeed, double accelerationdistance, double deccelerationdistance){
            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
            SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
            SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
            TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(), 1);
            telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
        }
        public void PowerSetting () {
            robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed + .15));
            robot.WB_M.setPower(Wobble.wobblePowerReturn());
            robot.GRIP_S.setPosition(Wobble.gripperSetReturn());
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