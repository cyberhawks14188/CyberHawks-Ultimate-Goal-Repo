
package org.firstinspires.ftc.teamcode;

import android.net.http.SslCertificate;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import com.qualcomm.robotcore.hardware.TouchSensor

public class RobotHardware{

    // Motors and Servos
    public DcMotor LF_M;
    public DcMotor LB_M;
    public DcMotor RF_M;
    public DcMotor RB_M;
    public DcMotor IN_M;
    public DcMotor STG_M;
    public DcMotor SOT_M;
    public DcMotor WB_M;
    public CRServo SOT_S;
    public Servo STOP_S;


    public AnalogInput WB_PT;
    public Servo GRIP_S;
    public AnalogInput SOT_PT;
    public NormalizedColorSensor Ring1_CS;
    public NormalizedColorSensor Ring2_CS;
    public NormalizedColorSensor Ring3_CS;
    public BNO055IMU imu;


    //Create Hardware map
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Define motors and servos
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        LF_M = hardwareMap.get(DcMotor.class, "LF_M");
        LB_M = hardwareMap.get(DcMotor.class, "LB_M");
        RF_M = hardwareMap.get(DcMotor.class, "RF_M");
        RB_M = hardwareMap.get(DcMotor.class, "RB_M");
        IN_M = hardwareMap.get(DcMotor.class, "IN_M");
        STG_M = hardwareMap.get(DcMotor.class, "STG_M");
        SOT_M = hardwareMap.get(DcMotor.class, "SOT_M");
        WB_M = hardwareMap.get(DcMotor.class, "WB_M");
        SOT_S = hardwareMap.get(CRServo.class, "SOT_S");
        GRIP_S = hardwareMap.get(Servo.class, "GRIP_S");
        SOT_PT = hardwareMap.get(AnalogInput.class, "SOT_PT");
        STOP_S = hardwareMap.get(Servo.class, "STOP_S");
        WB_PT = hardwareMap.get(AnalogInput.class, "WB_PT");
        Ring1_CS = hardwareMap.get(NormalizedColorSensor.class, "Ring1_CS");
        Ring2_CS = hardwareMap.get(NormalizedColorSensor.class, "Ring2_CS");
        Ring3_CS = hardwareMap.get(NormalizedColorSensor.class, "Ring3_CS");
        NormalizedColorSensor colorSensor;
        imu.initialize(parameters);
        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SOT_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SOT_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Zero power Behavor
        LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT
        LB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IN_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        STG_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SOT_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LF_M.setDirection(DcMotor.Direction.REVERSE);
        LB_M.setDirection(DcMotor.Direction.REVERSE);
        STG_M.setDirection(DcMotorSimple.Direction.REVERSE);

        LF_M.getCurrentPosition();
        LB_M.getCurrentPosition();
        RF_M.getCurrentPosition();

        // Define and initialize ALL installed servos.
        //servo.setPosition(0);
    }
}