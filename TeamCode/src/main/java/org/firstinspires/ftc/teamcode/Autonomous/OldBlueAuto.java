package org.firstinspires.ftc.teamcode.Autonomous;
//Imports RobotCore
import android.graphics.ImageDecoder;

import java.sql.Time;
import java.util.GregorianCalendar;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import com.qualcomm.robotcore.util.Range;
//Creates program Class
@Autonomous
//Sets name
//
public class OldBlueAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    //Sets up Vuforia
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Uses Vuforia Developer Code
    private static final String VUFORIA_KEY = "AZickLn/////AAABmRdNRU8Vt0+EsSkecZ/dEtdwfmReQRmGjONFJw9IrZwj83V0JqVOw7lVMu8esNz/c2srCeQNiZSotXn5mKGHThTl4m0nN9xTmOVBgIkUOrtkA1rGeUkBw0dPy5AD8pk5L4Mv2yikiYUEXDVsPvVYjsp9p2+SHZNPXSBRL8OUPsUa+DpTQnpdRgtca4ZmRFGwUsfqkj/2pTz3/aS8KpFzZ6mjMVKJbJwiZnMhND5Bhy600+NNUNiTka0g6E+9lDEBQI5H0XVkEGCjHIFA0F8Z7L4iIZhotBPNB8kx3ep3MSRQSGg/yrzNIM4av2BqM2JVohuQFh2uSWyDJdgEwxtZ6drh3YZIa12CsuSHNkgaas2k";
    //Declares Varibles
    public double x;
    public double y;
    public double z;
    public double z_error;
    public double x_error;
    public double z_porportional;
    public double x_porportional;
    public double previousTime;
    public double y_porportional;
    public double Z_Intergral;
    public double  X_Intergral;
    public double Y_Intergral;
    public double Z_Sum_of_Errors;
    public double lastDistanceFrom;
    double SOTCurrent;
    double While;
    public double X_Sum_of_Errors;
    double minimumVelocity;
    public double targetVelocity;
    double setVelocity;
    public double Y_Sum_of_Errors;
    public double Z_Last_Error;
    public double X_Last_Error;
    public double Y_Last_Error;
    public double Z_Diffrence_of_Errors;
    public double X_Diffrence_of_Errors;
    public double velocityDiffrenceOfErrors;
    public double velocityLastError;
    public double Y_Diffrence_of_Errors;
    public double VIM;
    public double velocityIntergral;
    public double Z_Derivitive;
    public double X_Derivitive;
    public double Y_Derivitive;
    public int moving;
    double GRIP_POS;
    public double z_encoder_diffrence;
    public double Distance;
    public double y_error;
    public double y_slope;
    public double x_slope;
    public double slope;
    public int last_Y_EndSetpoint;
    public double Z_PM;
    public double Z_IM;
    public double Z_DM;
    public double pass;
    public double X_PM;
    double Last_Y_EndSetpoint;
    double Last_X_EndSetpoint;
    public double X_IM;
    public double X_DM;
    public double Y_PM;
    public double Y_IM;
    public double Y_DM;
    public int last_X_EndSetpoint;
    double minimumAccelerationVelocity;
    public double Z_setpoint;
    public double Y_Distiance_From_Setpoint;
    public double X_EndSetpoint;
    public double VPM;
    public double velocityPorportion;
    double Y_Setpoint;
    double Slope_Y_error;
    double Slope_X_Error;
    double Slope_Y_porportional;
    double Slope_X_Porportional;
    double Slope_Y_Derivitive;
    double Slope_X_Derivitive;
    double Slope_Y_Correction;
    double Timedloop;
    double Slope_X_Correction;
    double X_Setpoint;
    public double expectedSpeedSetpoint;
    public double Y_EndSetpoint;
    public double LF_Direction;
    public double velocityError;
    public double LB_Direction;
    public double distanceWithin;
    public double actualVelocity;
    public double velocityCorrection;
    public double velocitySetpoint;
    public double E1;
    public double E2;
    double time_passed;
    public double E3;
    public double RF_Direction;
    public double RB_Direction;
    double JustTurn;
    double leaveLoop;
    public double Speed_Setpoint;
    public double Highest_Motor_Power;
    public double Distance_From ;
    double stopper;
    double stagerPower;
    double Slope_Y_PM;
    double shooterPM;
    double shooterActualVelocity;
    double shooterLastEncoder;
    double shooterError;
    double shooterSetpoint;
    double shooterPorportional;
    double shooterCorrection;
    double lastTime;
    double timepassed;
    double WB_Setpoint;
    double WB_error;
    double WB_PM;
    double Y_intercept;
    double Slope_Y_Last_errorl;
    double Slope_Y_Last_error;
    double Slope_Y_DM;
    double Slope_X_PM;
    double Slope_X_last_error;
    double Slope_X_DM;
    public double Y_Average;
    public double startPosition;
    public double Y_A2;
    public double velocitySumOfErrors;
    public int loopcount;
    double storingVarible1;
    double done;
    double storingVarible2;
    public int Slow_Down_Distance;
    double SOTSet;
    double maximumVelocity;
    double test;
    double SOTError;
    double shooterPower;
    double lastDistance;
    double SOTPower;
    double SOTP;
    double imuZ;
    public double veloictyDerivitive;
    double imuAngle;
    public double VDM;
    public double Slow_Rate;
    public double X_B2;
    public int breakout;
    public double runtime;
    public double Detected;
    double angles;
    public double Intial_Speed_Setpoint;
    //Makes the runOpMode public method
    public void runOpMode() {
        //Intializes our hardware map
        robot.init(hardwareMap);
        //Creates the Orintation Angles to be able to get the IMU's angle
        Orientation angles;
        //Resets Encoders
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //If we see no rings it will still be 0
        Detected = 0;

        //Intializes Vuforia
        initVuforia();
        //Intializes TensorFlow
        initTfod();
        //If TensorFlow is not active, activate it
        if (tfod != null) {
            tfod.activate();
        }
        //Will run until we hit play
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

                    telemetry();
                    telemetry.update();
                }
            }
        }

        //Waits for start
        waitForStart();
        //Our First move
        //Go up and to the left, just to the left of the ring stack
        shooterSetpoint = 0;
        SOTSet = 1.47;
        Distance_From = 1;
        WB_Setpoint = .32;
        GRIP_POS = .7;
        breakout = 1;
        targetVelocity = 37;
        while (Distance_From > .6 && opModeIsActive()) {
            Movement(-18, 47, 0, 6, 6);
            SubSystem();
        }
        stop_motors();
        //Low our wobble to just above the ground to make the dropping quicker
        WB_Setpoint = 1.3;
        //Split up our movements depending on how many rings were in the stack
        //If no rings go to Target Zone A
        if (Detected == 0) {
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 30;
            //Runs movement until 100 away
            while (Distance_From > .6 && opModeIsActive()) {
                Movement(-18, 64, 0, 6, 6);
                SubSystem();
            }
            stop_motors();

        }
        //If 1 ring go to target zone B
        if (Detected == 1) {
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 30;
            //Runs movement until 100 away

            while (Distance_From > .6 && opModeIsActive()) {
                Movement(8, 90, 0, 6, 6);
                SubSystem();
            }
            stop_motors();

        }
        //If 4 rings go to target zone C
        if (Detected == 2) {
            Last_X_EndSetpoint = storingVarible2;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 40;
            //Runs movement until 100 away
            while (Distance_From > .6 && opModeIsActive()) {
                Movement(-18, 112, 0, 5, 6);
                SubSystem();
            }
            stop_motors();

        }
        //Maintains our position while dropping the wobble goal and opening the gripper
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        WB_Setpoint = 2.04;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 5;
        shooterSetpoint = 1900;
        //velocitySetpoint = minimumVelocity;
        //Runs movement until 100 away
        while (robot.WB_PT.getVoltage() <= 2.02 && opModeIsActive()) {
            Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, 1, 1);
            SubSystem();
            if (robot.WB_PT.getVoltage() > 2) {
                GRIP_POS = .1;
            }
        }
        stop_motors();
        //Give the robot .5 seconds to finish dropping
        Timedloop = getRuntime() + .5;
        while(getRuntime() <= Timedloop){
            SubSystem();
        }
        //Moves our robot to shooting position
        GRIP_POS = 0.7;
        WB_Setpoint = .5;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 30;
        //Runs movement until 100 away
        while (Distance_From >= .6 && opModeIsActive()) {
            Movement(16, 60, 0, 6, 6);
            SubSystem();
        }
        //Maintains position to make sure our robot is lined up with the shooter
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 5;
        stop_motors();
        Timedloop = getRuntime() + .5;
        while(getRuntime() <= Timedloop && opModeIsActive()){
            SubSystem();
            Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, 1, 1);

        }
        //Begins shooting
        stop_motors();
        //Activates our stager to get the rings to the stagger motor
        stagerPower = -.7;
        //Lifts our stopper servo
        stopper = .5;
        //Turns on our shooter setpoint
        shooterSetpoint = 1900;
        //Shoots rings until our distance sensors sees that we have no rings in our stager
        /*while (robot.Ring1_DS.getDistance(DistanceUnit.INCH) < 2 || robot.Ring2_DS.getDistance(DistanceUnit.INCH) < 2 || robot.Ring3_DS.getDistance(DistanceUnit.INCH) < 4 && opModeIsActive()) {
            SubSystem();
        }
        */
        stop_motors();
        //Lowers wobble goal arm back down to grabbing position
        Timedloop = getRuntime() + .5;
        while(Timedloop>getRuntime()){
            SubSystem();
        }
        //Gives the robot .5 seconds to drop the wobble arm

        //Stops our shooter
        shooterSetpoint = 0;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 40;
        //Moves to the up and left to the 2nd wobble goal to be able to not hit the ring stack
        while (Distance_From > .6 && opModeIsActive()) {
            Movement(45, 35, 0, 6, 6);
            SubSystem();
        }
        //Stops our stagger
        WB_Setpoint = 2.04;
        stagerPower =0;
        stop_motors();
        //Opens our claw
        GRIP_POS = 0;

        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 30;
        //Moves to the wobble goal
        //Diffrent positions based on where our wobble goal is
        if(Detected == 0) {
            while (Distance_From > .6 && opModeIsActive()) {
                Movement(33, 22, 0, 6, 6);
                SubSystem();
            }
        }
        if(Detected == 1) {
            while (Distance_From > .6 && opModeIsActive()) {
                Movement(35, 22, 0, 6, 6);
                SubSystem();
            }
        }
        if(Detected == 2) {
            while (Distance_From > .6 && opModeIsActive()) {
                Movement(38.25, 22, 0, 6, 6);
                SubSystem();
            }
        }

        stop_motors();
        JustTurn = 1;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 20;
        //Turns the robot 86 degrees umping the IMU
        while (imuZ <= 86 && opModeIsActive()) {
            //angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //imuZ = angles.firstAngle;
            telemetry.addData("imuZ", imuZ);
            Movement(18.5, 42, -70, 1, 1);
            SubSystem();
        }
        stop_motors();
        Timedloop = ((robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2)+3.4;
        while(Timedloop >= Y_Average & opModeIsActive()){
            Y_Average = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            robot.LF_M.setPower(.3);
            robot.LB_M.setPower(.3);
            robot.RF_M.setPower(.3);
            robot.RB_M.setPower(.3);
        }
        stop_motors();
        //Closes claw
        GRIP_POS = .7;
        //Gives robot .75 seconds to Grab the 2nd wobble goal
        Timedloop = getRuntime() + .75;
        while(getRuntime() <= Timedloop){
            SubSystem();
        }
        //Raises wobble goal arm just above the ground
        WB_Setpoint = 1.2;
        Timedloop = getRuntime() + .75;
        while(getRuntime() <= Timedloop){
            SubSystem();
        }
        Timedloop = ((robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2)-3.4;
        while(Timedloop >= Y_Average & opModeIsActive()){
            Y_Average = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            robot.LF_M.setPower(-.3);
            robot.LB_M.setPower(-.3);
            robot.RF_M.setPower(-.3);
            robot.RB_M.setPower(-.3);
        }
        stop_motors();
        JustTurn = 1;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 20;
        //Turns the robot back forward using IMU
        while (imuZ >= 25 && opModeIsActive()) {
         //   angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          //  imuZ = angles.firstAngle;
            telemetry.addData("imuZ", imuZ);
            Movement(18.5, 42, 10, 1, 1);
            SubSystem();
        }
        stop_motors();
        JustTurn = 0;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 35;
        //Moves to a position where we can go to any target zone
        while (Distance_From > .5 && opModeIsActive()) {
            Movement(22, 40, 0, 6, 6);
            SubSystem();
        }
        //Depending on what the ring stack was we go back to the target zone where we dropped the 1st wobble goal
        if(Detected == 0){
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 30;

            while (Distance_From > .6 && opModeIsActive()) {
                Movement(-13, 60, 0, 6, 6);
                SubSystem();
            }
            stop_motors();
        }
        if(Detected == 1){
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 30;


            while (Distance_From > .6 && opModeIsActive()) {
                Movement(12, 86, 0, 6, 6);
                SubSystem();
            }
            stop_motors();
        }
        if (Detected == 2){
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 45;

            while (Distance_From > .8 && opModeIsActive()) {
                Movement(-12, 106, 0, 13, 9);
                SubSystem();
            }
            stop_motors();
        }
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        WB_Setpoint = 2.04;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = 5;
        //Drop the 2nd wobble goal and open GRIP_S
        Timedloop = getRuntime() + 2;
        while (robot.WB_PT.getVoltage() <= 2.02 && opModeIsActive()) {
            Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, 1, 1);
            SubSystem();
            if (robot.WB_PT.getVoltage() > 1.7) {
                GRIP_POS = .1;
            }
            if(getRuntime() >= Timedloop){
                break;
            }
        }

        stop_motors();
        Timedloop = getRuntime() + .5;
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 2;
        breakout = 1;
        targetVelocity = 40;

        //Drive backwards to the navigation line
        while (Distance_From > 1 && opModeIsActive()) {
            Movement(5, 72, 0, 6, 6);
            SubSystem();
        }
        stop_motors();


    }
    //Stops all drive motors
    public void stop_motors(){
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }

    //Displays useful telementry onto DS phone
    public void telemetry(){
        telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
        telemetry.addData("E1", robot.LF_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("E2", -robot.LB_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("E3", robot.RF_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("Y Slope Correction",Slope_Y_Correction);
        telemetry.addData("X Slope Corredton", Slope_X_Correction);
        telemetry.addData("Distance_From", Distance_From);
        telemetry.addData("Distance", Distance);
        telemetry.addData("Speed_Setpoint", Speed_Setpoint);
        telemetry.addData("Velocitiy Setpoint", velocitySetpoint);
        telemetry.addData("Velocity Correction", velocityCorrection);
        telemetry.addData("Actual Velocity", actualVelocity);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Y_EndSetpoint", Y_EndSetpoint);
        telemetry.addData("X_EndSetpoint", X_EndSetpoint);
        telemetry.addData("Last_Y_EndSetpoint", Last_Y_EndSetpoint);
        telemetry.addData("Last_X_EndSetpoint", Last_X_EndSetpoint);
        telemetry.addData("LF", robot.LF_M.getPower());
        telemetry.addData("LB", robot.LB_M.getPower());
        telemetry.addData("test", test);
        telemetry.addData("RF", robot.RF_M.getPower());
        telemetry.addData("slope", slope);
        telemetry.addData("RB", robot.RB_M.getPower());
        telemetry.addData("Dectected", Detected);

        telemetry.update();
    }

    //Uses a PID to move robot to XYZ setpoints
    public void Movement(double X_EndSetpoint, double Y_EndSetpoint, double Z_setpoint, double Slow_Down_Distance, double accelerationDistance) {
        //Sets our PID Multipliers
        X_PM = .15;
        X_IM = .000000001;
        X_DM = .35;
        Y_PM = .2;
        Y_IM = .000000001;
        Y_DM = .2;
        Z_PM = .45;
        Z_IM = .000000001;
        Z_DM = .55;
        VPM = .7;
        VIM = .00000001;
        VDM = .9;
        Slope_X_DM = .4;
        Slope_Y_PM = .4;
        Slope_X_PM = .4;
        Slope_Y_DM = .4;
        minimumAccelerationVelocity = 12;
        minimumVelocity = 2;
        //If we only want our robot to turn we only run the Z PID by setting all other multipliers to 0
        if(JustTurn == 1){
            X_PM = 0;
            X_IM = 0;
            X_DM = 0;
            Y_PM = 0;
            Y_IM = 0;
            Y_DM = 0;
            Slope_X_DM = 0;
            Slope_Y_PM = 0;
            Slope_X_PM = 0;
            Slope_Y_DM = 0;
            Z_PM = .15;
            Z_DM = .2;

        }

        //Gets encoder Positions
        //Converts our encoder ticks to inches
        E1 = robot.LF_M.getCurrentPosition() * 0.00436111;
        E2 = -robot.LB_M.getCurrentPosition() * 0.00436111;
        E3 = robot.RF_M.getCurrentPosition() * 0.00436111;
        //Sets encoders to .001 at begining to prevent null error
        if (E1 == 0) {
            E1 = .001;
        }
        if (E2 == 0) {
            E2 = .001;
        }

        if (E3 == 0) {
            E3 = .001;
        }
        //XYZ PID
        //Gets our robot moving in the direction of the endsetpoints


        //Finds the diffrence of E1 and E3
        z_encoder_diffrence = E1 - E3;
        //Finds the difference between our setpoint and our current position
        z_error = Z_setpoint - z_encoder_diffrence;
        //Z Porportional
        //Narrows down the rate of change by multiplying the error by a proportional multiplier
        z_porportional = Z_PM * z_error;
        // Z Intergral
        //Finds the Sum of the Errors
        Z_Sum_of_Errors = Z_Sum_of_Errors + z_error;
        Z_Intergral = Z_Sum_of_Errors * Z_IM;
        //Z Derivitive
        //Finds the difference between the current error and last loop's error
        Z_Diffrence_of_Errors = z_error - Z_Last_Error;
        Z_Derivitive = Z_Diffrence_of_Errors * Z_DM;
        Z_Last_Error = z_error;
        z = z_porportional + Z_Intergral + Z_Derivitive;
        //Finds the difference between our setpoint and our current position
        x_error = X_EndSetpoint - E2;
        //X Porportional
        //Narrows down the rate of change by multiplying the error by a proportional multiplier
        x_porportional = x_error * X_PM;
        //X Intergral
        //Finds the Sum of the Errors
        X_Sum_of_Errors = X_Sum_of_Errors + x_error;
        X_Intergral = X_Sum_of_Errors * X_IM;
        //X Derivitve
        //Finds the difference between the current error and last loop's error
        X_Diffrence_of_Errors = x_error - X_Last_Error;
        X_Derivitive = X_Diffrence_of_Errors * X_DM;
        X_Last_Error = x_error;

        //Finds Average Of E1 & E3
        Y_Average = (E1 + E3) / 2;
        //finds the average of E1 and E3
        //Finds the difference between our setpoint and our current position
        y_error = Y_EndSetpoint - Y_Average;
        //Y Porportional
        //Narrows down the rate of change by multiplying the error by a proportional multiplier
        y_porportional = Y_PM * y_error;
        // Y Intergral
        //Finds the Sum of the Errors
        Y_Sum_of_Errors = Y_Sum_of_Errors + y_error;
        Y_Intergral = Y_Sum_of_Errors * Y_IM;
        //Y Derivitive
        //Finds the difference between the current error and last loop's error
        Y_Diffrence_of_Errors = y_error - Y_Last_Error;
        Y_Derivitive = Y_Diffrence_of_Errors * Y_DM;
        Y_Last_Error = y_error;


        //Uses pythagorean therom to find how far we are from the end setpoints compared to our current position on the field our starting position
        Distance = Math.sqrt((Math.pow((Y_EndSetpoint-Last_Y_EndSetpoint), 2)) + (Math.pow((X_EndSetpoint-Last_X_EndSetpoint), 2)));
        //Calculates our Y_A2 by finding the difference between where we want to go and where we currently are
        Y_A2 = Y_EndSetpoint - Y_Average;
        //Calculates our X_B2 by finding the difference between where we want to go and where we currently are
        X_B2 = X_EndSetpoint - E2;
        //Calculates Distance from our end point from our current position
        Distance_From = Math.sqrt(Math.pow(Y_A2, 2) + (Math.pow(X_B2, 2)));
        //A if that only runs once, used to not allow our code to break out of the loop immediately
        if (breakout == 1) {
            lastDistanceFrom = Distance_From;
            breakout = 0;
        }

        //Gets our current run time
        time = getRuntime();
        //Finds the time passed from this loop cycle to the last loop cycle
        time_passed = time - (previousTime);
        //Velocity is the number of inches travled in a second
        actualVelocity = Math.abs((Distance_From - lastDistanceFrom)) / time_passed;
        //Sets previous time to time
        previousTime = time;
        //Sets lastDistanceFrom to our current Distance_From
        lastDistanceFrom = Distance_From;
        //Acceleration Code
        //Will run when our distance that we will accelerate for is greater than our our distance traveled
        if (accelerationDistance >= Distance - Distance_From) {
            //Creates a line using slope that our velocity setpoint
            //The farther we are from our starting position the more
            velocitySetpoint = ((Distance - Distance_From) * (targetVelocity / accelerationDistance));
            if(velocitySetpoint <= minimumAccelerationVelocity){
                velocitySetpoint = minimumAccelerationVelocity;
            }
        }
        //Run the deacceleration when our distance from is less then the Slow_Down_Distance
        if (Distance_From <= Slow_Down_Distance) {
            //Will ramp down from our target veloicty to 0 based on our distance
            velocitySetpoint = Distance_From * (targetVelocity / Slow_Down_Distance) + minimumVelocity;
        }
        //Allows us to call to a varible outside of the method
        storingVarible1 = Y_EndSetpoint;
        storingVarible2 = X_EndSetpoint;
        //Velocity limits
        if (velocitySetpoint <= minimumVelocity) {
            velocitySetpoint = minimumVelocity;
        }
        //Velcotiy Max speed per second in inches
        if (velocitySetpoint >= 50) {
            velocitySetpoint = 50;
        }
        //If our robot has stopped moving and we want to be moving then incress the velocity setpoint to get us to move
        if(actualVelocity == 0 && velocitySetpoint >.1){
            velocitySetpoint = velocitySetpoint + 4;
        }

        //Runs and PID on the velocity of our robot
        //Allows us to maintain the speed we want

        //Find the error between desired velcotiy and the current robot velocity
        velocityError = velocitySetpoint - actualVelocity;
        //Velocity Porportional
        velocityPorportion = velocityError * VPM;
        //Velocity Intergral
        velocitySumOfErrors = velocitySumOfErrors + velocityError;
        velocityIntergral = velocitySumOfErrors * VIM;
        //Velocity Derivitive
        velocityDiffrenceOfErrors = velocityError - velocityLastError;
        velocityLastError = velocityError;
        veloictyDerivitive = velocityDiffrenceOfErrors * VDM;
        //Finds the correction our robot has to do to get back to the desired setpoint
        velocityCorrection = (velocityPorportion + velocityIntergral + veloictyDerivitive);
        //Find our robot's slope based on our starting position to our end position
        slope = ((Y_EndSetpoint - Last_Y_EndSetpoint) / (X_EndSetpoint - Last_X_EndSetpoint));

        //Equate our maximum speed by the slope that we are going
        //Since when going sideways our robot can only go 30in/s and 50in/s when going forward
        //If our rise is higher than our run
        if(Math.abs(slope) >= 1) {
            maximumVelocity = 40 + (10-((1/Math.abs(slope))*10));
        }
        //If our run is higher then our rise
        else if(Math.abs(slope) < 1){
            maximumVelocity = 40 - (10-(Math.abs(slope) * 10));
        }
        //Finds our Y intercept
        Y_intercept = Y_EndSetpoint-(X_EndSetpoint*slope);
        if (Y_EndSetpoint - Last_Y_EndSetpoint != 0 && X_EndSetpoint - Last_X_EndSetpoint != 0) {
            Y_Setpoint = slope * E2+Y_intercept;
            X_Setpoint = (Y_Average-Y_intercept) / slope;
            telemetry.addData("1", "1");
        }
        //Horizonatal line
        //Have to do this since a horizontal lines slope is 0
        else if (Y_EndSetpoint - Last_Y_EndSetpoint == 0) {
            maximumVelocity = 30;
            Y_Setpoint = Y_EndSetpoint;
            X_Setpoint = E2;
            telemetry.addData("2", "2");
        }
        //Vertical Line
        //Have to do this since a vertical lines slope is undefined
        else if (X_EndSetpoint - Last_X_EndSetpoint == 0) {
            maximumVelocity = 50;
            Y_Setpoint = Y_Average;
            X_Setpoint = X_EndSetpoint;
            telemetry.addData("3", "3");
        }
        //When we are within 4 inches of target hold at the target instead of following the line
        if (Distance_From <= 4) {

            Y_Setpoint = Y_EndSetpoint;
            X_Setpoint = X_EndSetpoint;

        }
        //Find error between the desired point on line and our currenet E2 reading
        Slope_Y_error = Y_Setpoint - Y_Average;
        //Finds error between the desired point on line and our curernt E1 and E3 average
        Slope_X_Error = X_Setpoint - E2;
        //Runs the Y Slope PD
        Slope_Y_porportional = Slope_Y_error * Slope_Y_PM;
        Slope_Y_Derivitive = (Slope_Y_error - Slope_Y_Last_error) * Slope_Y_DM;
        Slope_Y_Last_error = Slope_Y_error;
        //Find the sum of the porportional and the derivitive
        Slope_Y_Correction = Slope_Y_porportional + Slope_Y_Derivitive;
        //Runs the X SLope PD
        Slope_X_Porportional = Slope_X_Error * Slope_X_PM;
        Slope_X_Derivitive = (Slope_X_Error - Slope_X_last_error) * Slope_X_DM;
        Slope_X_last_error = Slope_X_Error;
        //Find the sum of the  porprotional and the dervititive
        Slope_X_Correction = Slope_X_Porportional + Slope_X_Derivitive;

        //Equates our y direction correction
        y = y_porportional + Y_Intergral + Y_Derivitive + Slope_Y_Correction;
        //Equates our x direction correction
        x = x_porportional + X_Intergral + X_Derivitive + Slope_X_Correction;
        //Runs our motor equation method
        MotorEquation();
        //Runs our telementry method
        telemetry();
    }

    public void MotorEquation() {
        //Motor equation from the slope and XYZ pid
        LF_Direction = y+(x+z);
        LB_Direction = y-(x-z);
        RF_Direction = y-(x+z);
        RB_Direction = y+(x-z);
        //Finds Highest power out of the drive motors
        //We do this to be able to then get a -1-1 reading on our motors
        Highest_Motor_Power = Math.max(Math.max(Math.abs(RF_Direction), Math.abs(RB_Direction)), Math.max(Math.abs(LF_Direction), Math.abs(LB_Direction)));
        //Sets motors
        //Determines our speed of our motors by the velocity correction and setpoint
        robot.LF_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (LF_Direction/Highest_Motor_Power));
        robot.LB_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (LB_Direction/Highest_Motor_Power));
        robot.RF_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (RF_Direction/Highest_Motor_Power));
        robot.RB_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (RB_Direction/Highest_Motor_Power));
    }

    //Method for our mechanisms other than our drivebase
    public void SubSystem() {
        //Gets our current potentiometer reading from our shooter angle
        SOTCurrent = robot.SOT_PT.getVoltage();
        //Sets our proportional multipliers for our sub system correctional loops
        SOTP = -20;
        WB_PM = .6;
        shooterPM = 15;
        //Shooter servo correectional loop
        SOTError = SOTSet - SOTCurrent;
        SOTPower = SOTError * SOTP;
        //Gets our wobble goal error from our setpoint
        WB_error = WB_Setpoint - robot.WB_PT.getVoltage();
        //Only runs our shooter Proportional loop when we have a setpoint greater than 1
        if (shooterSetpoint > 0){
            //Runs a velocity correction loop on our shooter motor
            //Get the time passed for our velocity
            timepassed = getRuntime() - lastTime;
            //Get our shooter velocity using our distance/time, where distance is the encoder's ticks per loop cycle
            shooterActualVelocity = Math.abs(robot.SOT_M.getCurrentPosition() - shooterLastEncoder) / timepassed;
            lastTime = getRuntime();
            //Sets our current encoder's to our last encoders
            shooterLastEncoder = robot.SOT_M.getCurrentPosition();
            //Find the error between our velocity setpoint and our current velocity
            shooterError = shooterSetpoint - shooterActualVelocity;
            shooterPorportional = shooterError * shooterPM;
            //Finds the correction needed to maintain velocity
            shooterCorrection = shooterPorportional;
        }
        //IF we do not want shooter to run, don't let it run
        else if(shooterSetpoint == 0) {
            shooterCorrection = 0;
            shooterSetpoint = 0;
        }
        //Sets our subsystem powers
        robot.WB_M.setPower(WB_error*WB_PM);
        robot.SOT_M.setPower((shooterCorrection + shooterSetpoint)/2800);
        robot.GRIP_S.setPosition(GRIP_POS);
        robot.SOT_S.setPower(SOTPower);
        robot.STOP_S.setPosition(stopper);
        robot.STG_M.setPower(-stagerPower);

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