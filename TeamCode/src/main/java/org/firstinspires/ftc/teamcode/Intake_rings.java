package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class Intake_rings extends LinearOpMode {
     double SOTCurrent;
    double SOTP = -20;
    double WB_PM = .4;
    double SOTPower;
    double SOTSet = 2.24;
    double SOTError;

    public  void runOpMode(){
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            SOTCurrent =  robot.SOT_PT.getVoltage();
            SOTError = SOTSet - SOTCurrent;
            SOTPower = SOTError * SOTP;

            if(SOTCurrent < SOTSet){
                robot.SOT_S.setPower(SOTPower);
            }
            else{
                robot.SOT_S.setPower(0);
            }
            robot.IN_M.setPower(-1);
            robot.STG_M.setPower(1);
            robot.STOP_S.setPosition(.3);
        }
        robot.IN_M.setPower(0);
        robot.STG_M.setPower(0);
    }
}
