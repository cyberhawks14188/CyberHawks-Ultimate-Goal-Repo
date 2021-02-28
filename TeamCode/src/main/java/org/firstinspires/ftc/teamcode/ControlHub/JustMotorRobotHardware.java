
package org.firstinspires.ftc.teamcode.ControlHub;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor

public class JustMotorRobotHardware {

    // Motors and Servos
    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public DcMotor Motor4;
    public NormalizedColorSensor ColorSensor1;


    //Create Hardware map
    HardwareMap JustmotorhardwareMap;

    public void init(HardwareMap JustmotorhardwareMap) {


        // Define motors and servos
        Motor1 = JustmotorhardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = JustmotorhardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = JustmotorhardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = JustmotorhardwareMap.get(DcMotor.class, "Motor4");
        ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");
        NormalizedColorSensor colorSensor;



        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);







        // Define and initialize ALL installed servos.
        //servo.setPosition(0);
    }
}