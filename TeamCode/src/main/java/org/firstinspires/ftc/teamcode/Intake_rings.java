package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class Intake_rings extends LinearOpMode {
     double SOTCurrent;
    double SOTP = -20;
    double WB_PM = .4;
    double SOTPower;
    double SOTSet = 1.6;
    double SOTError;

    public  void runOpMode(){
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        waitForStart();
        robot.Ring3_CS.setGain(15);
        robot.Ring2_CS.setGain(15);
        robot.Ring1_CS.setGain(15);
        while(opModeIsActive()) {
            NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
            NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
            NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();

            SOTCurrent =  robot.SOT_PT.getVoltage();
            SOTError = SOTSet - SOTCurrent;
            SOTPower = SOTError * SOTP;

            if(SOTCurrent < SOTSet){
                robot.SOT_S.setPower(SOTPower);
            }
            else {
                robot.SOT_S.setPower(0);
            }
            if (Ring1Color.red > .05 && Ring2Color.red > .25 && Ring3Color.red > .25) {
                robot.IN_M.setPower(0);
                robot.STG_M.setPower(0);
            }else{
                robot.IN_M.setPower(-1);
                robot.STG_M.setPower(1);
            }

            robot.STOP_S.setPosition(.3);
        }
        robot.IN_M.setPower(0);
        robot.STG_M.setPower(0);
    }
}
