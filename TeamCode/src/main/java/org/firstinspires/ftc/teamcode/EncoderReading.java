package org.firstinspires.ftc.teamcode;
//Imports RobotCore

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Creates program Class
@Autonomous
//Sets name
public class EncoderReading extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    public void runOpMode() {
        robot.init(hardwareMap);
        //Resets Encoders
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("E1", robot.LF_M.getCurrentPosition());
            telemetry.addData("E2", robot.LB_M.getCurrentPosition());
            telemetry.addData("E3", robot.RF_M.getCurrentPosition());
            telemetry.update();

        }
    }
}