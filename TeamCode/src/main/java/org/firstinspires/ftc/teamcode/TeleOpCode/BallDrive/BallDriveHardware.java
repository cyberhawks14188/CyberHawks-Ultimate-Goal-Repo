
package org.firstinspires.ftc.teamcode.TeleOpCode.BallDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor

public class BallDriveHardware {

    // Motors and Servos
    public DcMotor RB_X_M;
    public DcMotor RB_Y_M;
    public DcMotor LB_X_M;
    public DcMotor LB_Y_M;


    //Create Hardware map
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // Define motors and servos

        LB_X_M = hardwareMap.get(DcMotor.class, "LB_X_M");
        LB_Y_M = hardwareMap.get(DcMotor.class, "LB_Y_M");
        RB_X_M = hardwareMap.get(DcMotor.class, "RB_X_M");
        RB_Y_M = hardwareMap.get(DcMotor.class, "RB_Y_M");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LB_X_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_Y_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_X_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_Y_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_X_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_Y_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_X_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_Y_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Zero power Behavor
        //LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //or FLOAT


        // Define and initialize ALL installed servos.
        //servo.setPosition(0);
    }
}